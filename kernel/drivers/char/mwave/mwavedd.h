

#ifndef _LINUX_MWAVEDD_H
#define _LINUX_MWAVEDD_H
#include "3780i.h"
#include "tp3780i.h"
#include "smapi.h"
#include "mwavepub.h"
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/wait.h>

extern int mwave_debug;
extern int mwave_3780i_irq;
extern int mwave_3780i_io;
extern int mwave_uart_irq;
extern int mwave_uart_io;

#define PRINTK_ERROR printk
#define KERN_ERR_MWAVE KERN_ERR "mwave: "

#define TRACE_MWAVE     0x0001
#define TRACE_SMAPI     0x0002
#define TRACE_3780I     0x0004
#define TRACE_TP3780I   0x0008

#ifdef MW_TRACE
#define PRINTK_1(f,s)                       \
  if (f & (mwave_debug)) {                  \
    printk(s);                              \
  }

#define PRINTK_2(f,s,v1)                    \
  if (f & (mwave_debug)) {                  \
    printk(s,v1);                           \
  }

#define PRINTK_3(f,s,v1,v2)                 \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2);                        \
  }

#define PRINTK_4(f,s,v1,v2,v3)              \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2,v3);                     \
  }

#define PRINTK_5(f,s,v1,v2,v3,v4)           \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2,v3,v4);                  \
  }

#define PRINTK_6(f,s,v1,v2,v3,v4,v5)        \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2,v3,v4,v5);               \
  }

#define PRINTK_7(f,s,v1,v2,v3,v4,v5,v6)     \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2,v3,v4,v5,v6);            \
  }

#define PRINTK_8(f,s,v1,v2,v3,v4,v5,v6,v7)  \
  if (f & (mwave_debug)) {                  \
    printk(s,v1,v2,v3,v4,v5,v6,v7);         \
  }

#else
#define PRINTK_1(f,s)
#define PRINTK_2(f,s,v1)
#define PRINTK_3(f,s,v1,v2)
#define PRINTK_4(f,s,v1,v2,v3)
#define PRINTK_5(f,s,v1,v2,v3,v4)
#define PRINTK_6(f,s,v1,v2,v3,v4,v5)
#define PRINTK_7(f,s,v1,v2,v3,v4,v5,v6)
#define PRINTK_8(f,s,v1,v2,v3,v4,v5,v6,v7)
#endif


typedef struct _MWAVE_IPC {
	unsigned short usIntCount;	/* 0=none, 1=first, 2=greater than 1st */
	BOOLEAN bIsEnabled;
	BOOLEAN bIsHere;
	/* entry spin lock */
	wait_queue_head_t ipc_wait_queue;
} MWAVE_IPC;

typedef struct _MWAVE_DEVICE_DATA {
	THINKPAD_BD_DATA rBDData;	/* board driver's data area */
	unsigned long ulIPCSource_ISR;	/* IPC source bits for recently processed intr, set during ISR processing */
	unsigned long ulIPCSource_DPC;	/* IPC source bits for recently processed intr, set during DPC processing */
	BOOLEAN bBDInitialized;
	BOOLEAN bResourcesClaimed;
	BOOLEAN bDSPEnabled;
	BOOLEAN bDSPReset;
	MWAVE_IPC IPCs[16];
	BOOLEAN bMwaveDevRegistered;
	short sLine;
	int nr_registered_attrs;
	int device_registered;

} MWAVE_DEVICE_DATA, *pMWAVE_DEVICE_DATA;

extern MWAVE_DEVICE_DATA mwave_s_mdd;

#endif
