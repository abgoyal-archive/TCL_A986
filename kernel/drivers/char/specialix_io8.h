

#ifndef __LINUX_SPECIALIX_H
#define __LINUX_SPECIALIX_H

#include <linux/serial.h>

#ifdef __KERNEL__


#define SX_NBOARD		8

/* NOTE: Specialix decoder recognizes 4 addresses, but only two are used.... */
#define SX_IO_SPACE             4
/* The PCI version decodes 8 addresses, but still only 2 are used. */
#define SX_PCI_IO_SPACE         8

/* eight ports per board. */
#define SX_NPORT        	8
#define SX_BOARD(line)		((line) / SX_NPORT)
#define SX_PORT(line)		((line) & (SX_NPORT - 1))


#define SX_DATA_REG 0     /* Base+0 : Data register */
#define SX_ADDR_REG 1     /* base+1 : Address register. */

#define MHz *1000000	/* I'm ashamed of myself. */

/* On-board oscillator frequency */
#define SX_OSCFREQ      (25 MHz/2)
/* There is a 25MHz crystal on the board, but the chip is in /2 mode */


/* Ticks per sec. Used for setting receiver timeout and break length */
#define SPECIALIX_TPS		4000

#define SPECIALIX_RXFIFO	6	/* Max. receiver FIFO size (1-8) */

#define SPECIALIX_MAGIC		0x0907

#define SX_CCR_TIMEOUT 10000   /* CCR timeout. You may need to wait upto
                                  10 milliseconds before the internal
                                  processor is available again after
                                  you give it a command */

#define SX_IOBASE1	0x100
#define SX_IOBASE2	0x180
#define SX_IOBASE3	0x250
#define SX_IOBASE4	0x260

struct specialix_board {
	unsigned long   flags;
	unsigned short	base;
	unsigned char 	irq;
	//signed   char	count;
	int count;
	unsigned char	DTR;
        int reg;
	spinlock_t lock;
};

#define SX_BOARD_PRESENT	0x00000001
#define SX_BOARD_ACTIVE		0x00000002
#define SX_BOARD_IS_PCI		0x00000004


struct specialix_port {
	int			magic;
	struct tty_port		port;
	int			baud_base;
	int			flags;
	int			timeout;
	unsigned char 		* xmit_buf;
	int			custom_divisor;
	int			xmit_head;
	int			xmit_tail;
	int			xmit_cnt;
	short			wakeup_chars;
	short			break_length;
	unsigned char		mark_mask;
	unsigned char		IER;
	unsigned char		MSVR;
	unsigned char		COR2;
	unsigned long		overrun;
	unsigned long		hits[10];
	spinlock_t lock;
};

#endif /* __KERNEL__ */
#endif /* __LINUX_SPECIALIX_H */









