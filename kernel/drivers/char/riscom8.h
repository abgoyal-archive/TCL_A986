

#ifndef __LINUX_RISCOM8_H
#define __LINUX_RISCOM8_H

#include <linux/serial.h>

#ifdef __KERNEL__

#define RC_NBOARD		4
/* NOTE: RISCom decoder recognizes 16 addresses... */
#define RC_NPORT        	8  
#define RC_BOARD(line)		(((line) >> 3) & 0x07)
#define RC_PORT(line)		((line) & (RC_NPORT - 1))

/* Ticks per sec. Used for setting receiver timeout and break length */
#define RISCOM_TPS		4000

#define RISCOM_RXFIFO		6	/* Max. receiver FIFO size (1-8) */

#define RISCOM8_MAGIC		0x0907

#define RC_IOBASE1	0x220
#define RC_IOBASE2	0x240
#define RC_IOBASE3	0x250
#define RC_IOBASE4	0x260

struct riscom_board {
	unsigned long   flags;
	unsigned short	base;
	unsigned char 	irq;
	signed   char	count;
	unsigned char	DTR;
};

#define RC_BOARD_PRESENT	0x00000001
#define RC_BOARD_ACTIVE		0x00000002
	
struct riscom_port {
	int			magic;
	struct			tty_port port;
	int			baud_base;
	int			timeout;
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
#ifdef RC_REPORT_OVERRUN
	unsigned long		overrun;
#endif	
#ifdef RC_REPORT_FIFO
	unsigned long		hits[10];
#endif
};

#endif /* __KERNEL__ */
#endif /* __LINUX_RISCOM8_H */
