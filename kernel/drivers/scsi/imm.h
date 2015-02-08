


#ifndef _IMM_H
#define _IMM_H

#define   IMM_VERSION   "2.05 (for Linux 2.4.0)"

/* ------ END OF USER CONFIGURABLE PARAMETERS ----- */

#include  <linux/stddef.h>
#include  <linux/module.h>
#include  <linux/kernel.h>
#include  <linux/ioport.h>
#include  <linux/delay.h>
#include  <linux/proc_fs.h>
#include  <linux/stat.h>
#include  <linux/blkdev.h>
#include  <linux/sched.h>
#include  <linux/interrupt.h>

#include  <asm/io.h>
#include  <scsi/scsi_host.h>
/* batteries not included :-) */

#define   IMM_AUTODETECT        0	/* Autodetect mode                */
#define   IMM_NIBBLE            1	/* work in standard 4 bit mode    */
#define   IMM_PS2               2	/* PS/2 byte mode         */
#define   IMM_EPP_8             3	/* EPP mode, 8 bit                */
#define   IMM_EPP_16            4	/* EPP mode, 16 bit               */
#define   IMM_EPP_32            5	/* EPP mode, 32 bit               */
#define   IMM_UNKNOWN           6	/* Just in case...                */

static char *IMM_MODE_STRING[] =
{
	[IMM_AUTODETECT] = "Autodetect",
	[IMM_NIBBLE]	 = "SPP",
	[IMM_PS2]	 = "PS/2",
	[IMM_EPP_8]	 = "EPP 8 bit",
	[IMM_EPP_16]	 = "EPP 16 bit",
#ifdef CONFIG_SCSI_IZIP_EPP16
	[IMM_EPP_32]	 = "EPP 16 bit",
#else
	[IMM_EPP_32]	 = "EPP 32 bit",
#endif
	[IMM_UNKNOWN]	 = "Unknown",
};

/* other options */
#define IMM_BURST_SIZE	512	/* data burst size */
#define IMM_SELECT_TMO  500	/* 500 how long to wait for target ? */
#define IMM_SPIN_TMO    5000	/* 50000 imm_wait loop limiter */
#define IMM_DEBUG	0	/* debugging option */
#define IN_EPP_MODE(x) (x == IMM_EPP_8 || x == IMM_EPP_16 || x == IMM_EPP_32)

/* args to imm_connect */
#define CONNECT_EPP_MAYBE 1
#define CONNECT_NORMAL  0

#define r_dtr(x)        (unsigned char)inb((x))
#define r_str(x)        (unsigned char)inb((x)+1)
#define r_ctr(x)        (unsigned char)inb((x)+2)
#define r_epp(x)        (unsigned char)inb((x)+4)
#define r_fifo(x)       (unsigned char)inb((x))   /* x must be base_hi */
					/* On PCI is: base+0x400 != base_hi */
#define r_ecr(x)        (unsigned char)inb((x)+2) /* x must be base_hi */

#define w_dtr(x,y)      outb(y, (x))
#define w_str(x,y)      outb(y, (x)+1)
#define w_epp(x,y)      outb(y, (x)+4)
#define w_fifo(x,y)     outb(y, (x))     /* x must be base_hi */
#define w_ecr(x,y)      outb(y, (x)+0x2) /* x must be base_hi */

#ifdef CONFIG_SCSI_IZIP_SLOW_CTR
#define w_ctr(x,y)      outb_p(y, (x)+2)
#else
#define w_ctr(x,y)      outb(y, (x)+2)
#endif

static int imm_engine(imm_struct *, struct scsi_cmnd *);

#endif				/* _IMM_H */
