
#ifndef _LINUX_LP_H
#define _LINUX_LP_H


#define LP_EXIST 0x0001
#define LP_SELEC 0x0002
#define LP_BUSY	 0x0004
#define LP_BUSY_BIT_POS 2
#define LP_OFFL	 0x0008
#define LP_NOPA  0x0010
#define LP_ERR   0x0020
#define LP_ABORT 0x0040
#define LP_CAREFUL 0x0080 /* obsoleted -arca */
#define LP_ABORTOPEN 0x0100

#define LP_TRUST_IRQ_  0x0200 /* obsolete */
#define LP_NO_REVERSE  0x0400 /* No reverse mode available. */
#define LP_DATA_AVAIL  0x0800 /* Data is available. */

#define LP_PBUSY	0x80  /* inverted input, active high */
#define LP_PACK		0x40  /* unchanged input, active low */
#define LP_POUTPA	0x20  /* unchanged input, active high */
#define LP_PSELECD	0x10  /* unchanged input, active high */
#define LP_PERRORP	0x08  /* unchanged input, active low */


#define LP_INIT_CHAR 1000


#define LP_INIT_WAIT 1


#define LP_INIT_TIME 2

/* IOCTL numbers */
#define LPCHAR   0x0601  /* corresponds to LP_INIT_CHAR */
#define LPTIME   0x0602  /* corresponds to LP_INIT_TIME */
#define LPABORT  0x0604  /* call with TRUE arg to abort on error,
			    FALSE to retry.  Default is retry.  */
#define LPSETIRQ 0x0605  /* call with new IRQ number,
			    or 0 for polling (no IRQ) */
#define LPGETIRQ 0x0606  /* get the current IRQ number */
#define LPWAIT   0x0608  /* corresponds to LP_INIT_WAIT */
/* NOTE: LPCAREFUL is obsoleted and it' s always the default right now -arca */
#define LPCAREFUL   0x0609  /* call with TRUE arg to require out-of-paper, off-
			    line, and error indicators good on all writes,
			    FALSE to ignore them.  Default is ignore. */
#define LPABORTOPEN 0x060a  /* call with TRUE arg to abort open() on error,
			    FALSE to ignore error.  Default is ignore.  */
#define LPGETSTATUS 0x060b  /* return LP_S(minor) */
#define LPRESET     0x060c  /* reset printer */
#ifdef LP_STATS
#define LPGETSTATS  0x060d  /* get statistics (struct lp_stats) */
#endif
#define LPGETFLAGS  0x060e  /* get status flags */
#define LPSETTIMEOUT 0x060f /* set parport timeout */


#define LP_TIMEOUT_INTERRUPT	(60 * HZ)
#define LP_TIMEOUT_POLLED	(10 * HZ)

#ifdef __KERNEL__

#include <linux/wait.h>
#include <linux/mutex.h>

/* Magic numbers for defining port-device mappings */
#define LP_PARPORT_UNSPEC -4
#define LP_PARPORT_AUTO -3
#define LP_PARPORT_OFF -2
#define LP_PARPORT_NONE -1

#define LP_F(minor)	lp_table[(minor)].flags		/* flags for busy, etc. */
#define LP_CHAR(minor)	lp_table[(minor)].chars		/* busy timeout */
#define LP_TIME(minor)	lp_table[(minor)].time		/* wait time */
#define LP_WAIT(minor)	lp_table[(minor)].wait		/* strobe wait */
#define LP_IRQ(minor)	lp_table[(minor)].dev->port->irq /* interrupt # */
					/* PARPORT_IRQ_NONE means polled */
#ifdef LP_STATS
#define LP_STAT(minor)	lp_table[(minor)].stats		/* statistics area */
#endif
#define LP_BUFFER_SIZE PAGE_SIZE

#define LP_BASE(x)	lp_table[(x)].dev->port->base

#ifdef LP_STATS
struct lp_stats {
	unsigned long chars;
	unsigned long sleeps;
	unsigned int maxrun;
	unsigned int maxwait;
	unsigned int meanwait;
	unsigned int mdev;
};
#endif

struct lp_struct {
	struct pardevice *dev;
	unsigned long flags;
	unsigned int chars;
	unsigned int time;
	unsigned int wait;
	char *lp_buffer;
#ifdef LP_STATS
	unsigned int lastcall;
	unsigned int runchars;
	struct lp_stats stats;
#endif
	wait_queue_head_t waitq;
	unsigned int last_error;
	struct mutex port_mutex;
	wait_queue_head_t dataq;
	long timeout;
	unsigned int best_mode;
	unsigned int current_mode;
	unsigned long bits;
};



#define LP_PINTEN	0x10  /* high to read data in or-ed with data out */
#define LP_PSELECP	0x08  /* inverted output, active low */
#define LP_PINITP	0x04  /* unchanged output, active low */
#define LP_PAUTOLF	0x02  /* inverted output, active low */
#define LP_PSTROBE	0x01  /* short high output on raising edge */

#define LP_DUMMY	0x00

#define LP_DELAY 	50

#endif

#endif
