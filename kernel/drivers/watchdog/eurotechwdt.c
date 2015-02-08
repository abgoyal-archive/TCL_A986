



#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/system.h>

static unsigned long eurwdt_is_open;
static int eurwdt_timeout;
static char eur_expect_close;
static spinlock_t eurwdt_lock;


static int io = 0x3f0;
static int irq = 10;
static char *ev = "int";

#define WDT_TIMEOUT		60                /* 1 minute */

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");


#define WDT_CTRL_REG		0x30
#define WDT_OUTPIN_CFG		0xe2
#define WDT_EVENT_INT		0x00
#define WDT_EVENT_REBOOT	0x08
#define WDT_UNIT_SEL		0xf1
#define WDT_UNIT_SECS		0x80
#define WDT_TIMEOUT_VAL		0xf2
#define WDT_TIMER_CFG		0xf3


module_param(io, int, 0);
MODULE_PARM_DESC(io, "Eurotech WDT io port (default=0x3f0)");
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "Eurotech WDT irq (default=10)");
module_param(ev, charp, 0);
MODULE_PARM_DESC(ev, "Eurotech WDT event type (default is `int')");



static inline void eurwdt_write_reg(u8 index, u8 data)
{
	outb(index, io);
	outb(data, io+1);
}

static inline void eurwdt_lock_chip(void)
{
	outb(0xaa, io);
}

static inline void eurwdt_unlock_chip(void)
{
	outb(0x55, io);
	eurwdt_write_reg(0x07, 0x08);	/* set the logical device */
}

static inline void eurwdt_set_timeout(int timeout)
{
	eurwdt_write_reg(WDT_TIMEOUT_VAL, (u8) timeout);
}

static inline void eurwdt_disable_timer(void)
{
	eurwdt_set_timeout(0);
}

static void eurwdt_activate_timer(void)
{
	eurwdt_disable_timer();
	eurwdt_write_reg(WDT_CTRL_REG, 0x01);	/* activate the WDT */
	eurwdt_write_reg(WDT_OUTPIN_CFG,
		!strcmp("int", ev) ? WDT_EVENT_INT : WDT_EVENT_REBOOT);

	/* Setting interrupt line */
	if (irq == 2 || irq > 15 || irq < 0) {
		printk(KERN_ERR ": invalid irq number\n");
		irq = 0;	/* if invalid we disable interrupt */
	}
	if (irq == 0)
		printk(KERN_INFO ": interrupt disabled\n");

	eurwdt_write_reg(WDT_TIMER_CFG, irq << 4);

	eurwdt_write_reg(WDT_UNIT_SEL, WDT_UNIT_SECS);	/* we use seconds */
	eurwdt_set_timeout(0);	/* the default timeout */
}



static irqreturn_t eurwdt_interrupt(int irq, void *dev_id)
{
	printk(KERN_CRIT "timeout WDT timeout\n");

#ifdef ONLY_TESTING
	printk(KERN_CRIT "Would Reboot.\n");
#else
	printk(KERN_CRIT "Initiating system reboot.\n");
	emergency_restart();
#endif
	return IRQ_HANDLED;
}



static void eurwdt_ping(void)
{
	/* Write the watchdog default value */
	eurwdt_set_timeout(eurwdt_timeout);
}


static ssize_t eurwdt_write(struct file *file, const char __user *buf,
size_t count, loff_t *ppos)
{
	if (count) 	{
		if (!nowayout) {
			size_t i;

			eur_expect_close = 0;

			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, buf + i))
					return -EFAULT;
				if (c == 'V')
					eur_expect_close = 42;
			}
		}
		spin_lock(&eurwdt_lock);
		eurwdt_ping();	/* the default timeout */
		spin_unlock(&eurwdt_lock);
	}
	return count;
}


static long eurwdt_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	static const struct watchdog_info ident = {
		.options	  = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT
							| WDIOF_MAGICCLOSE,
		.firmware_version = 1,
		.identity	  = "WDT Eurotech CPU-1220/1410",
	};

	int time;
	int options, retval = -EINVAL;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_SETOPTIONS:
		if (get_user(options, p))
			return -EFAULT;
		spin_lock(&eurwdt_lock);
		if (options & WDIOS_DISABLECARD) {
			eurwdt_disable_timer();
			retval = 0;
		}
		if (options & WDIOS_ENABLECARD) {
			eurwdt_activate_timer();
			eurwdt_ping();
			retval = 0;
		}
		spin_unlock(&eurwdt_lock);
		return retval;

	case WDIOC_KEEPALIVE:
		spin_lock(&eurwdt_lock);
		eurwdt_ping();
		spin_unlock(&eurwdt_lock);
		return 0;

	case WDIOC_SETTIMEOUT:
		if (copy_from_user(&time, p, sizeof(int)))
			return -EFAULT;

		/* Sanity check */
		if (time < 0 || time > 255)
			return -EINVAL;

		spin_lock(&eurwdt_lock);
		eurwdt_timeout = time;
		eurwdt_set_timeout(time);
		spin_unlock(&eurwdt_lock);
		/* Fall */

	case WDIOC_GETTIMEOUT:
		return put_user(eurwdt_timeout, p);

	default:
		return -ENOTTY;
	}
}


static int eurwdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &eurwdt_is_open))
		return -EBUSY;
	eurwdt_timeout = WDT_TIMEOUT;	/* initial timeout */
	/* Activate the WDT */
	eurwdt_activate_timer();
	return nonseekable_open(inode, file);
}


static int eurwdt_release(struct inode *inode, struct file *file)
{
	if (eur_expect_close == 42)
		eurwdt_disable_timer();
	else {
		printk(KERN_CRIT
			"eurwdt: Unexpected close, not stopping watchdog!\n");
		eurwdt_ping();
	}
	clear_bit(0, &eurwdt_is_open);
	eur_expect_close = 0;
	return 0;
}


static int eurwdt_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		eurwdt_disable_timer();	/* Turn the card off */

	return NOTIFY_DONE;
}



static const struct file_operations eurwdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= eurwdt_write,
	.unlocked_ioctl	= eurwdt_ioctl,
	.open		= eurwdt_open,
	.release	= eurwdt_release,
};

static struct miscdevice eurwdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "watchdog",
	.fops	= &eurwdt_fops,
};


static struct notifier_block eurwdt_notifier = {
	.notifier_call = eurwdt_notify_sys,
};


static void __exit eurwdt_exit(void)
{
	eurwdt_lock_chip();

	misc_deregister(&eurwdt_miscdev);

	unregister_reboot_notifier(&eurwdt_notifier);
	release_region(io, 2);
	free_irq(irq, NULL);
}


static int __init eurwdt_init(void)
{
	int ret;

	ret = request_irq(irq, eurwdt_interrupt, IRQF_DISABLED, "eurwdt", NULL);
	if (ret) {
		printk(KERN_ERR "eurwdt: IRQ %d is not free.\n", irq);
		goto out;
	}

	if (!request_region(io, 2, "eurwdt")) {
		printk(KERN_ERR "eurwdt: IO %X is not free.\n", io);
		ret = -EBUSY;
		goto outirq;
	}

	ret = register_reboot_notifier(&eurwdt_notifier);
	if (ret) {
		printk(KERN_ERR
		    "eurwdt: can't register reboot notifier (err=%d)\n", ret);
		goto outreg;
	}

	spin_lock_init(&eurwdt_lock);

	ret = misc_register(&eurwdt_miscdev);
	if (ret) {
		printk(KERN_ERR "eurwdt: can't misc_register on minor=%d\n",
		WATCHDOG_MINOR);
		goto outreboot;
	}

	eurwdt_unlock_chip();

	ret = 0;
	printk(KERN_INFO "Eurotech WDT driver 0.01 at %X (Interrupt %d)"
		" - timeout event: %s\n",
		io, irq, (!strcmp("int", ev) ? "int" : "reboot"));

out:
	return ret;

outreboot:
	unregister_reboot_notifier(&eurwdt_notifier);

outreg:
	release_region(io, 2);

outirq:
	free_irq(irq, NULL);
	goto out;
}

module_init(eurwdt_init);
module_exit(eurwdt_exit);

MODULE_AUTHOR("Rodolfo Giometti");
MODULE_DESCRIPTION("Driver for Eurotech CPU-1220/1410 on board watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
