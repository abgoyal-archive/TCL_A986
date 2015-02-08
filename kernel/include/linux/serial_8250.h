
#ifndef _LINUX_SERIAL_8250_H
#define _LINUX_SERIAL_8250_H

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct plat_serial8250_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	resource_size_t	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned long	irqflags;	/* request_irq flags */
	unsigned int	uartclk;	/* UART clock rate */
	void            *private_data;
	unsigned char	regshift;	/* register shift */
	unsigned char	iotype;		/* UPIO_* */
	unsigned char	hub6;
	upf_t		flags;		/* UPF_* flags */
	unsigned int	type;		/* If UPF_FIXED_TYPE */
	unsigned int	(*serial_in)(struct uart_port *, int);
	void		(*serial_out)(struct uart_port *, int, int);
};

enum {
	PLAT8250_DEV_LEGACY = -1,
	PLAT8250_DEV_PLATFORM,
	PLAT8250_DEV_PLATFORM1,
	PLAT8250_DEV_PLATFORM2,
	PLAT8250_DEV_FOURPORT,
	PLAT8250_DEV_ACCENT,
	PLAT8250_DEV_BOCA,
	PLAT8250_DEV_EXAR_ST16C554,
	PLAT8250_DEV_HUB6,
	PLAT8250_DEV_MCA,
	PLAT8250_DEV_AU1X00,
	PLAT8250_DEV_SM501,
};

struct uart_port;

int serial8250_register_port(struct uart_port *);
void serial8250_unregister_port(int line);
void serial8250_suspend_port(int line);
void serial8250_resume_port(int line);

extern int early_serial_setup(struct uart_port *port);

extern int serial8250_find_port(struct uart_port *p);
extern int serial8250_find_port_for_earlycon(void);
extern int setup_early_serial8250_console(char *cmdline);

#endif
