
/*  Module Name : wrap_dbg.c                                            */
/*                                                                      */
/*  Abstract                                                            */
/*     This module contains wrapper functions for debug functions       */
/*                                                                      */
/*  NOTES                                                               */
/*     Platform dependent.                                              */
/*                                                                      */
/************************************************************************/

#include "oal_dt.h"
#include "usbdrv.h"

#include <linux/netlink.h>
#include <net/iw_handler.h>

void zfwDumpBuf(zdev_t *dev, zbuf_t *buf)
{
	u16_t i;

	for (i = 0; i < buf->len; i++) {
		printk(KERN_DEBUG "%02x ", *(((u8_t *)buf->data)+i));
		if ((i & 0xf) == 0xf)
			printk(KERN_DEBUG "\n");
	}
	printk(KERN_DEBUG "\n");
}


void zfwDbgReadRegDone(zdev_t *dev, u32_t addr, u32_t val)
{
	printk(KERN_DEBUG "Read addr:%x = %x\n", addr, val);
}

void zfwDbgWriteRegDone(zdev_t *dev, u32_t addr, u32_t val)
{
	printk(KERN_DEBUG "Write addr:%x = %x\n", addr, val);
}

void zfwDbgReadTallyDone(zdev_t *dev)
{
	/* printk(KERN_DEBUG "Read Tall Done\n"); */
}

void zfwDbgWriteEepromDone(zdev_t *dev, u32_t addr, u32_t val)
{
}

void zfwDbgQueryHwTxBusyDone(zdev_t *dev, u32_t val)
{
}

/* For Evl ++ */
void zfwDbgReadFlashDone(zdev_t *dev, u32_t addr, u32_t *rspdata, u32_t datalen)
{
	printk(KERN_DEBUG "Read Flash addr:%x length:%x\n", addr, datalen);
}

void zfwDbgProgrameFlashDone(zdev_t *dev)
{
	printk(KERN_DEBUG "Program Flash Done\n");
}

void zfwDbgProgrameFlashChkDone(zdev_t *dev)
{
	printk(KERN_DEBUG "Program Flash Done\n");
}

void zfwDbgGetFlashChkSumDone(zdev_t *dev, u32_t *rspdata)
{
	printk(KERN_DEBUG "Get Flash ChkSum Done\n");
}

void zfwDbgDownloadFwInitDone(zdev_t *dev)
{
	printk(KERN_DEBUG "Download FW Init Done\n");
}
/* For Evl -- */

/* Leave an empty line below to remove warning message on some compiler */
