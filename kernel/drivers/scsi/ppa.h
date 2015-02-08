

#ifndef _PPA_H
#define _PPA_H

#define   PPA_VERSION   "2.07 (for Linux 2.4.x)"

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

#define   PPA_AUTODETECT        0	/* Autodetect mode                */
#define   PPA_NIBBLE            1	/* work in standard 4 bit mode    */
#define   PPA_PS2               2	/* PS/2 byte mode         */
#define   PPA_EPP_8             3	/* EPP mode, 8 bit                */
#define   PPA_EPP_16            4	/* EPP mode, 16 bit               */
#define   PPA_EPP_32            5	/* EPP mode, 32 bit               */
#define   PPA_UNKNOWN           6	/* Just in case...                */

static char *PPA_MODE_STRING[] =
{
    "Autodetect",
    "SPP",
    "PS/2",
    "EPP 8 bit",
    "EPP 16 bit",
#ifdef CONFIG_SCSI_IZIP_EPP16
    "EPP 16 bit",
#else
    "EPP 32 bit",
#endif
    "Unknown"};

/* other options */
#define PPA_BURST_SIZE	512	/* data burst size */
#define PPA_SELECT_TMO  5000	/* how long to wait for target ? */
#define PPA_SPIN_TMO    50000	/* ppa_wait loop limiter */
#define PPA_RECON_TMO   500	/* scsi reconnection loop limiter */
#define PPA_DEBUG	0	/* debugging option */
#define IN_EPP_MODE(x) (x == PPA_EPP_8 || x == PPA_EPP_16 || x == PPA_EPP_32)

/* args to ppa_connect */
#define CONNECT_EPP_MAYBE 1
#define CONNECT_NORMAL  0

#define r_dtr(x)        (unsigned char)inb((x))
#define r_str(x)        (unsigned char)inb((x)+1)
#define r_ctr(x)        (unsigned char)inb((x)+2)
#define r_epp(x)        (unsigned char)inb((x)+4)
#define r_fifo(x)       (unsigned char)inb((x)) /* x must be base_hi */
					/* On PCI is base+0x400 != base_hi */
#define r_ecr(x)        (unsigned char)inb((x)+0x2) /* x must be base_hi */

#define w_dtr(x,y)      outb(y, (x))
#define w_str(x,y)      outb(y, (x)+1)
#define w_epp(x,y)      outb(y, (x)+4)
#define w_fifo(x,y)     outb(y, (x))	/* x must be base_hi */
#define w_ecr(x,y)      outb(y, (x)+0x2)/* x must be base_hi */

#ifdef CONFIG_SCSI_IZIP_SLOW_CTR
#define w_ctr(x,y)      outb_p(y, (x)+2)
#else
#define w_ctr(x,y)      outb(y, (x)+2)
#endif

static int ppa_engine(ppa_struct *, struct scsi_cmnd *);

#endif				/* _PPA_H */
