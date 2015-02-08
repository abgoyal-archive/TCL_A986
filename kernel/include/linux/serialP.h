

#ifndef _LINUX_SERIALP_H
#define _LINUX_SERIALP_H


#include <linux/termios.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/circ_buf.h>
#include <linux/wait.h>

struct serial_state {
	int	magic;
	int	baud_base;
	unsigned long	port;
	int	irq;
	int	flags;
	int	hub6;
	int	type;
	int	line;
	int	revision;	/* Chip revision (950) */
	int	xmit_fifo_size;
	int	custom_divisor;
	int	count;
	u8	*iomem_base;
	u16	iomem_reg_shift;
	unsigned short	close_delay;
	unsigned short	closing_wait; /* time to wait before closing */
	struct async_icount	icount;	
	int	io_type;
	struct async_struct *info;
	struct pci_dev	*dev;
};

struct async_struct {
	int			magic;
	unsigned long		port;
	int			hub6;
	int			flags;
	int			xmit_fifo_size;
	struct serial_state	*state;
	struct tty_struct 	*tty;
	int			read_status_mask;
	int			ignore_status_mask;
	int			timeout;
	int			quot;
	int			x_char;	/* xon/xoff character */
	int			close_delay;
	unsigned short		closing_wait;
	unsigned short		closing_wait2; /* obsolete */
	int			IER; 	/* Interrupt Enable Register */
	int			MCR; 	/* Modem control register */
	int			LCR; 	/* Line control register */
	int			ACR;	 /* 16950 Additional Control Reg. */
	unsigned long		event;
	unsigned long		last_active;
	int			line;
	int			blocked_open; /* # of blocked opens */
 	struct circ_buf		xmit;
 	spinlock_t		xmit_lock;
	u8			*iomem_base;
	u16			iomem_reg_shift;
	int			io_type;
	struct work_struct			work;
	struct tasklet_struct	tlet;
#ifdef DECLARE_WAITQUEUE
	wait_queue_head_t	open_wait;
	wait_queue_head_t	close_wait;
	wait_queue_head_t	delta_msr_wait;
#else	
	struct wait_queue	*open_wait;
	struct wait_queue	*close_wait;
	struct wait_queue	*delta_msr_wait;
#endif	
	struct async_struct	*next_port; /* For the linked list */
	struct async_struct	*prev_port;
};

#define CONFIGURED_SERIAL_PORT(info) ((info)->port || ((info)->iomem_base))

#define SERIAL_MAGIC 0x5301
#define SSTATE_MAGIC 0x5302

#define RS_EVENT_WRITE_WAKEUP	0

struct rs_multiport_struct {
	int		port1;
	unsigned char	mask1, match1;
	int		port2;
	unsigned char	mask2, match2;
	int		port3;
	unsigned char	mask3, match3;
	int		port4;
	unsigned char	mask4, match4;
	int		port_monitor;
};

#if defined(__alpha__) && !defined(CONFIG_PCI)
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#elif defined(CONFIG_SBC8560)
#define ALPHA_KLUDGE_MCR (UART_MCR_OUT2)
#else
#define ALPHA_KLUDGE_MCR 0
#endif

#endif /* _LINUX_SERIAL_H */
