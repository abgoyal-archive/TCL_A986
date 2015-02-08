

#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/sysrq.h>
#include <linux/circ_buf.h>
#include <linux/serial_reg.h>
#include <linux/delay.h> /* for mdelay */
#include <linux/miscdevice.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/sn/simulator.h>
#include <asm/sn/sn_sal.h>

/* number of characters we can transmit to the SAL console at a time */
#define SN_SAL_MAX_CHARS 120

#define SN_SAL_BUFFER_SIZE (64 * (1 << 10))

#define SN_SAL_UART_FIFO_DEPTH 16
#define SN_SAL_UART_FIFO_SPEED_CPS (9600/10)

/* sn_transmit_chars() calling args */
#define TRANSMIT_BUFFERED	0
#define TRANSMIT_RAW		1

				  /* #define USE_DYNAMIC_MINOR 1 *//* use dynamic minor number */
#define USE_DYNAMIC_MINOR 0	/* Don't rely on misc_register dynamic minor */

/* Device name we're using */
#define DEVICE_NAME "ttySG"
#define DEVICE_NAME_DYNAMIC "ttySG0"	/* need full name for misc_register */
/* The major/minor we are using, ignored for USE_DYNAMIC_MINOR */
#define DEVICE_MAJOR 204
#define DEVICE_MINOR 40

#ifdef CONFIG_MAGIC_SYSRQ
static char sysrq_serial_str[] = "\eSYS";
static char *sysrq_serial_ptr = sysrq_serial_str;
static unsigned long sysrq_requested;
#endif /* CONFIG_MAGIC_SYSRQ */

struct sn_cons_port {
	struct timer_list sc_timer;
	struct uart_port sc_port;
	struct sn_sal_ops {
		int (*sal_puts_raw) (const char *s, int len);
		int (*sal_puts) (const char *s, int len);
		int (*sal_getc) (void);
		int (*sal_input_pending) (void);
		void (*sal_wakeup_transmit) (struct sn_cons_port *, int);
	} *sc_ops;
	unsigned long sc_interrupt_timeout;
	int sc_is_asynch;
};

static struct sn_cons_port sal_console_port;
static int sn_process_input;

/* Only used if USE_DYNAMIC_MINOR is set to 1 */
static struct miscdevice misc;	/* used with misc_register for dynamic */

extern void early_sn_setup(void);

#undef DEBUG
#ifdef DEBUG
static int sn_debug_printf(const char *fmt, ...);
#define DPRINTF(x...) sn_debug_printf(x)
#else
#define DPRINTF(x...) do { } while (0)
#endif

/* Prototypes */
static int snt_hw_puts_raw(const char *, int);
static int snt_hw_puts_buffered(const char *, int);
static int snt_poll_getc(void);
static int snt_poll_input_pending(void);
static int snt_intr_getc(void);
static int snt_intr_input_pending(void);
static void sn_transmit_chars(struct sn_cons_port *, int);

static struct sn_sal_ops poll_ops = {
	.sal_puts_raw = snt_hw_puts_raw,
	.sal_puts = snt_hw_puts_raw,
	.sal_getc = snt_poll_getc,
	.sal_input_pending = snt_poll_input_pending
};

/* A table for interrupts enabled */
static struct sn_sal_ops intr_ops = {
	.sal_puts_raw = snt_hw_puts_raw,
	.sal_puts = snt_hw_puts_buffered,
	.sal_getc = snt_intr_getc,
	.sal_input_pending = snt_intr_input_pending,
	.sal_wakeup_transmit = sn_transmit_chars
};


/* routines for running the console in polling mode */

static int snt_poll_getc(void)
{
	int ch;

	ia64_sn_console_getc(&ch);
	return ch;
}

static int snt_poll_input_pending(void)
{
	int status, input;

	status = ia64_sn_console_check(&input);
	return !status && input;
}

/* routines for an interrupt driven console (normal) */

static int snt_intr_getc(void)
{
	return ia64_sn_console_readc();
}

static int snt_intr_input_pending(void)
{
	return ia64_sn_console_intr_status() & SAL_CONSOLE_INTR_RECV;
}

/* these functions are polled and interrupt */

static int snt_hw_puts_raw(const char *s, int len)
{
	/* this will call the PROM and not return until this is done */
	return ia64_sn_console_putb(s, len);
}

static int snt_hw_puts_buffered(const char *s, int len)
{
	/* queue data to the PROM */
	return ia64_sn_console_xmit_chars((char *)s, len);
}


static const char *snp_type(struct uart_port *port)
{
	return ("SGI SN L1");
}

static unsigned int snp_tx_empty(struct uart_port *port)
{
	return 1;
}

static void snp_stop_tx(struct uart_port *port)
{
}

static void snp_release_port(struct uart_port *port)
{
}

static void snp_enable_ms(struct uart_port *port)
{
}

static void snp_shutdown(struct uart_port *port)
{
}

static void snp_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int snp_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_RNG | TIOCM_DSR | TIOCM_CTS;
}

static void snp_stop_rx(struct uart_port *port)
{
}

static void snp_start_tx(struct uart_port *port)
{
	if (sal_console_port.sc_ops->sal_wakeup_transmit)
		sal_console_port.sc_ops->sal_wakeup_transmit(&sal_console_port,
							     TRANSMIT_BUFFERED);

}

static void snp_break_ctl(struct uart_port *port, int break_state)
{
}

static int snp_startup(struct uart_port *port)
{
	return 0;
}

static void
snp_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
}

static int snp_request_port(struct uart_port *port)
{
	return 0;
}

static void snp_config_port(struct uart_port *port, int flags)
{
}

/* Associate the uart functions above - given to serial core */

static struct uart_ops sn_console_ops = {
	.tx_empty = snp_tx_empty,
	.set_mctrl = snp_set_mctrl,
	.get_mctrl = snp_get_mctrl,
	.stop_tx = snp_stop_tx,
	.start_tx = snp_start_tx,
	.stop_rx = snp_stop_rx,
	.enable_ms = snp_enable_ms,
	.break_ctl = snp_break_ctl,
	.startup = snp_startup,
	.shutdown = snp_shutdown,
	.set_termios = snp_set_termios,
	.pm = NULL,
	.type = snp_type,
	.release_port = snp_release_port,
	.request_port = snp_request_port,
	.config_port = snp_config_port,
	.verify_port = NULL,
};

/* End of uart struct functions and defines */

#ifdef DEBUG

static int sn_debug_printf(const char *fmt, ...)
{
	static char printk_buf[1024];
	int printed_len;
	va_list args;

	va_start(args, fmt);
	printed_len = vsnprintf(printk_buf, sizeof(printk_buf), fmt, args);

	if (!sal_console_port.sc_ops) {
		sal_console_port.sc_ops = &poll_ops;
		early_sn_setup();
	}
	sal_console_port.sc_ops->sal_puts_raw(printk_buf, printed_len);

	va_end(args);
	return printed_len;
}
#endif				/* DEBUG */


static void
sn_receive_chars(struct sn_cons_port *port, unsigned long flags)
{
	int ch;
	struct tty_struct *tty;

	if (!port) {
		printk(KERN_ERR "sn_receive_chars - port NULL so can't receieve\n");
		return;
	}

	if (!port->sc_ops) {
		printk(KERN_ERR "sn_receive_chars - port->sc_ops  NULL so can't receieve\n");
		return;
	}

	if (port->sc_port.state) {
		/* The serial_core stuffs are initilized, use them */
		tty = port->sc_port.state->port.tty;
	}
	else {
		/* Not registered yet - can't pass to tty layer.  */
		tty = NULL;
	}

	while (port->sc_ops->sal_input_pending()) {
		ch = port->sc_ops->sal_getc();
		if (ch < 0) {
			printk(KERN_ERR "sn_console: An error occured while "
			       "obtaining data from the console (0x%0x)\n", ch);
			break;
		}
#ifdef CONFIG_MAGIC_SYSRQ
                if (sysrq_requested) {
                        unsigned long sysrq_timeout = sysrq_requested + HZ*5;

                        sysrq_requested = 0;
                        if (ch && time_before(jiffies, sysrq_timeout)) {
                                spin_unlock_irqrestore(&port->sc_port.lock, flags);
                                handle_sysrq(ch, NULL);
                                spin_lock_irqsave(&port->sc_port.lock, flags);
                                /* ignore actual sysrq command char */
                                continue;
                        }
                }
                if (ch == *sysrq_serial_ptr) {
                        if (!(*++sysrq_serial_ptr)) {
                                sysrq_requested = jiffies;
                                sysrq_serial_ptr = sysrq_serial_str;
                        }
			/*
			 * ignore the whole sysrq string except for the
			 * leading escape
			 */
			if (ch != '\e')
				continue;
                }
                else
			sysrq_serial_ptr = sysrq_serial_str;
#endif /* CONFIG_MAGIC_SYSRQ */

		/* record the character to pass up to the tty layer */
		if (tty) {
			if(tty_insert_flip_char(tty, ch, TTY_NORMAL) == 0)
				break;
		}
		port->sc_port.icount.rx++;
	}

	if (tty)
		tty_flip_buffer_push(tty);
}

static void sn_transmit_chars(struct sn_cons_port *port, int raw)
{
	int xmit_count, tail, head, loops, ii;
	int result;
	char *start;
	struct circ_buf *xmit;

	if (!port)
		return;

	BUG_ON(!port->sc_is_asynch);

	if (port->sc_port.state) {
		/* We're initilized, using serial core infrastructure */
		xmit = &port->sc_port.state->xmit;
	} else {
		/* Probably sn_sal_switch_to_asynch has been run but serial core isn't
		 * initilized yet.  Just return.  Writes are going through
		 * sn_sal_console_write (due to register_console) at this time.
		 */
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->sc_port)) {
		/* Nothing to do. */
		ia64_sn_console_intr_disable(SAL_CONSOLE_INTR_XMIT);
		return;
	}

	head = xmit->head;
	tail = xmit->tail;
	start = &xmit->buf[tail];

	/* twice around gets the tail to the end of the buffer and
	 * then to the head, if needed */
	loops = (head < tail) ? 2 : 1;

	for (ii = 0; ii < loops; ii++) {
		xmit_count = (head < tail) ?
		    (UART_XMIT_SIZE - tail) : (head - tail);

		if (xmit_count > 0) {
			if (raw == TRANSMIT_RAW)
				result =
				    port->sc_ops->sal_puts_raw(start,
							       xmit_count);
			else
				result =
				    port->sc_ops->sal_puts(start, xmit_count);
#ifdef DEBUG
			if (!result)
				DPRINTF("`");
#endif
			if (result > 0) {
				xmit_count -= result;
				port->sc_port.icount.tx += result;
				tail += result;
				tail &= UART_XMIT_SIZE - 1;
				xmit->tail = tail;
				start = &xmit->buf[tail];
			}
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&port->sc_port);

	if (uart_circ_empty(xmit))
		snp_stop_tx(&port->sc_port);	/* no-op for us */
}

static irqreturn_t sn_sal_interrupt(int irq, void *dev_id)
{
	struct sn_cons_port *port = (struct sn_cons_port *)dev_id;
	unsigned long flags;
	int status = ia64_sn_console_intr_status();

	if (!port)
		return IRQ_NONE;

	spin_lock_irqsave(&port->sc_port.lock, flags);
	if (status & SAL_CONSOLE_INTR_RECV) {
		sn_receive_chars(port, flags);
	}
	if (status & SAL_CONSOLE_INTR_XMIT) {
		sn_transmit_chars(port, TRANSMIT_BUFFERED);
	}
	spin_unlock_irqrestore(&port->sc_port.lock, flags);
	return IRQ_HANDLED;
}

static void sn_sal_timer_poll(unsigned long data)
{
	struct sn_cons_port *port = (struct sn_cons_port *)data;
	unsigned long flags;

	if (!port)
		return;

	if (!port->sc_port.irq) {
		spin_lock_irqsave(&port->sc_port.lock, flags);
		if (sn_process_input)
			sn_receive_chars(port, flags);
		sn_transmit_chars(port, TRANSMIT_RAW);
		spin_unlock_irqrestore(&port->sc_port.lock, flags);
		mod_timer(&port->sc_timer,
			  jiffies + port->sc_interrupt_timeout);
	}
}


static void __init sn_sal_switch_to_asynch(struct sn_cons_port *port)
{
	unsigned long flags;

	if (!port)
		return;

	DPRINTF("sn_console: about to switch to asynchronous console\n");

	/* without early_printk, we may be invoked late enough to race
	 * with other cpus doing console IO at this point, however
	 * console interrupts will never be enabled */
	spin_lock_irqsave(&port->sc_port.lock, flags);

	/* early_printk invocation may have done this for us */
	if (!port->sc_ops)
		port->sc_ops = &poll_ops;

	/* we can't turn on the console interrupt (as request_irq
	 * calls kmalloc, which isn't set up yet), so we rely on a
	 * timer to poll for input and push data from the console
	 * buffer.
	 */
	init_timer(&port->sc_timer);
	port->sc_timer.function = sn_sal_timer_poll;
	port->sc_timer.data = (unsigned long)port;

	if (IS_RUNNING_ON_SIMULATOR())
		port->sc_interrupt_timeout = 6;
	else {
		/* 960cps / 16 char FIFO = 60HZ
		 * HZ / (SN_SAL_FIFO_SPEED_CPS / SN_SAL_FIFO_DEPTH) */
		port->sc_interrupt_timeout =
		    HZ * SN_SAL_UART_FIFO_DEPTH / SN_SAL_UART_FIFO_SPEED_CPS;
	}
	mod_timer(&port->sc_timer, jiffies + port->sc_interrupt_timeout);

	port->sc_is_asynch = 1;
	spin_unlock_irqrestore(&port->sc_port.lock, flags);
}

static void __init sn_sal_switch_to_interrupts(struct sn_cons_port *port)
{
	unsigned long flags;

	if (port) {
		DPRINTF("sn_console: switching to interrupt driven console\n");

		if (request_irq(SGI_UART_VECTOR, sn_sal_interrupt,
				IRQF_DISABLED | IRQF_SHARED,
				"SAL console driver", port) >= 0) {
			spin_lock_irqsave(&port->sc_port.lock, flags);
			port->sc_port.irq = SGI_UART_VECTOR;
			port->sc_ops = &intr_ops;

			/* turn on receive interrupts */
			ia64_sn_console_intr_enable(SAL_CONSOLE_INTR_RECV);
			spin_unlock_irqrestore(&port->sc_port.lock, flags);
		}
		else {
			printk(KERN_INFO
			    "sn_console: console proceeding in polled mode\n");
		}
	}
}


static void sn_sal_console_write(struct console *, const char *, unsigned);
static int sn_sal_console_setup(struct console *, char *);
static struct uart_driver sal_console_uart;
extern struct tty_driver *uart_console_device(struct console *, int *);

static struct console sal_console = {
	.name = DEVICE_NAME,
	.write = sn_sal_console_write,
	.device = uart_console_device,
	.setup = sn_sal_console_setup,
	.index = -1,		/* unspecified */
	.data = &sal_console_uart,
};

#define SAL_CONSOLE	&sal_console

static struct uart_driver sal_console_uart = {
	.owner = THIS_MODULE,
	.driver_name = "sn_console",
	.dev_name = DEVICE_NAME,
	.major = 0,		/* major/minor set at registration time per USE_DYNAMIC_MINOR */
	.minor = 0,
	.nr = 1,		/* one port */
	.cons = SAL_CONSOLE,
};

static int __init sn_sal_module_init(void)
{
	int retval;

	if (!ia64_platform_is("sn2"))
		return 0;

	printk(KERN_INFO "sn_console: Console driver init\n");

	if (USE_DYNAMIC_MINOR == 1) {
		misc.minor = MISC_DYNAMIC_MINOR;
		misc.name = DEVICE_NAME_DYNAMIC;
		retval = misc_register(&misc);
		if (retval != 0) {
			printk(KERN_WARNING "Failed to register console "
			       "device using misc_register.\n");
			return -ENODEV;
		}
		sal_console_uart.major = MISC_MAJOR;
		sal_console_uart.minor = misc.minor;
	} else {
		sal_console_uart.major = DEVICE_MAJOR;
		sal_console_uart.minor = DEVICE_MINOR;
	}

	/* We register the driver and the port before switching to interrupts
	 * or async above so the proper uart structures are populated */

	if (uart_register_driver(&sal_console_uart) < 0) {
		printk
		    ("ERROR sn_sal_module_init failed uart_register_driver, line %d\n",
		     __LINE__);
		return -ENODEV;
	}

	spin_lock_init(&sal_console_port.sc_port.lock);

	/* Setup the port struct with the minimum needed */
	sal_console_port.sc_port.membase = (char *)1;	/* just needs to be non-zero */
	sal_console_port.sc_port.type = PORT_16550A;
	sal_console_port.sc_port.fifosize = SN_SAL_MAX_CHARS;
	sal_console_port.sc_port.ops = &sn_console_ops;
	sal_console_port.sc_port.line = 0;

	if (uart_add_one_port(&sal_console_uart, &sal_console_port.sc_port) < 0) {
		/* error - not sure what I'd do - so I'll do nothing */
		printk(KERN_ERR "%s: unable to add port\n", __func__);
	}

	/* when this driver is compiled in, the console initialization
	 * will have already switched us into asynchronous operation
	 * before we get here through the module initcalls */
	if (!sal_console_port.sc_is_asynch) {
		sn_sal_switch_to_asynch(&sal_console_port);
	}

	/* at this point (module_init) we can try to turn on interrupts */
	if (!IS_RUNNING_ON_SIMULATOR()) {
		sn_sal_switch_to_interrupts(&sal_console_port);
	}
	sn_process_input = 1;
	return 0;
}

static void __exit sn_sal_module_exit(void)
{
	del_timer_sync(&sal_console_port.sc_timer);
	uart_remove_one_port(&sal_console_uart, &sal_console_port.sc_port);
	uart_unregister_driver(&sal_console_uart);
	misc_deregister(&misc);
}

module_init(sn_sal_module_init);
module_exit(sn_sal_module_exit);


static void puts_raw_fixed(int (*puts_raw) (const char *s, int len),
			   const char *s, int count)
{
	const char *s1;

	/* Output '\r' before each '\n' */
	while ((s1 = memchr(s, '\n', count)) != NULL) {
		puts_raw(s, s1 - s);
		puts_raw("\r\n", 2);
		count -= s1 + 1 - s;
		s = s1 + 1;
	}
	puts_raw(s, count);
}

static void
sn_sal_console_write(struct console *co, const char *s, unsigned count)
{
	unsigned long flags = 0;
	struct sn_cons_port *port = &sal_console_port;
	static int stole_lock = 0;

	BUG_ON(!port->sc_is_asynch);

	/* We can't look at the xmit buffer if we're not registered with serial core
	 *  yet.  So only do the fancy recovery after registering
	 */
	if (!port->sc_port.state) {
		/* Not yet registered with serial core - simple case */
		puts_raw_fixed(port->sc_ops->sal_puts_raw, s, count);
		return;
	}

	/* somebody really wants this output, might be an
	 * oops, kdb, panic, etc.  make sure they get it. */
	if (spin_is_locked(&port->sc_port.lock)) {
		int lhead = port->sc_port.state->xmit.head;
		int ltail = port->sc_port.state->xmit.tail;
		int counter, got_lock = 0;

		/*
		 * We attempt to determine if someone has died with the
		 * lock. We wait ~20 secs after the head and tail ptrs
		 * stop moving and assume the lock holder is not functional
		 * and plow ahead. If the lock is freed within the time out
		 * period we re-get the lock and go ahead normally. We also
		 * remember if we have plowed ahead so that we don't have
		 * to wait out the time out period again - the asumption
		 * is that we will time out again.
		 */

		for (counter = 0; counter < 150; mdelay(125), counter++) {
			if (!spin_is_locked(&port->sc_port.lock)
			    || stole_lock) {
				if (!stole_lock) {
					spin_lock_irqsave(&port->sc_port.lock,
							  flags);
					got_lock = 1;
				}
				break;
			} else {
				/* still locked */
				if ((lhead != port->sc_port.state->xmit.head)
				    || (ltail !=
					port->sc_port.state->xmit.tail)) {
					lhead =
						port->sc_port.state->xmit.head;
					ltail =
						port->sc_port.state->xmit.tail;
					counter = 0;
				}
			}
		}
		/* flush anything in the serial core xmit buffer, raw */
		sn_transmit_chars(port, 1);
		if (got_lock) {
			spin_unlock_irqrestore(&port->sc_port.lock, flags);
			stole_lock = 0;
		} else {
			/* fell thru */
			stole_lock = 1;
		}
		puts_raw_fixed(port->sc_ops->sal_puts_raw, s, count);
	} else {
		stole_lock = 0;
		spin_lock_irqsave(&port->sc_port.lock, flags);
		sn_transmit_chars(port, 1);
		spin_unlock_irqrestore(&port->sc_port.lock, flags);

		puts_raw_fixed(port->sc_ops->sal_puts_raw, s, count);
	}
}


static int sn_sal_console_setup(struct console *co, char *options)
{
	return 0;
}

static void __init
sn_sal_console_write_early(struct console *co, const char *s, unsigned count)
{
	puts_raw_fixed(sal_console_port.sc_ops->sal_puts_raw, s, count);
}

static struct console sal_console_early __initdata = {
	.name = "sn_sal",
	.write = sn_sal_console_write_early,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};

int __init sn_serial_console_early_setup(void)
{
	if (!ia64_platform_is("sn2"))
		return -1;

	sal_console_port.sc_ops = &poll_ops;
	spin_lock_init(&sal_console_port.sc_port.lock);
	early_sn_setup();	/* Find SAL entry points */
	register_console(&sal_console_early);

	return 0;
}

static int __init sn_sal_serial_console_init(void)
{
	if (ia64_platform_is("sn2")) {
		sn_sal_switch_to_asynch(&sal_console_port);
		DPRINTF("sn_sal_serial_console_init : register console\n");
		register_console(&sal_console);
		unregister_console(&sal_console_early);
	}
	return 0;
}

console_initcall(sn_sal_serial_console_init);
