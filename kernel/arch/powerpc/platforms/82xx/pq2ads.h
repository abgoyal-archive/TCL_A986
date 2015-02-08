

#ifdef __KERNEL__
#ifndef __MACH_ADS8260_DEFS
#define __MACH_ADS8260_DEFS

#include <linux/seq_file.h>


#define BCSR0_LED0		((uint)0x02000000)      /* 0 == on */
#define BCSR0_LED1		((uint)0x01000000)      /* 0 == on */
#define BCSR1_FETHIEN		((uint)0x08000000)      /* 0 == enable*/
#define BCSR1_FETH_RST		((uint)0x04000000)      /* 0 == reset */
#define BCSR1_RS232_EN1		((uint)0x02000000)      /* 0 ==enable */
#define BCSR1_RS232_EN2		((uint)0x01000000)      /* 0 ==enable */
#define BCSR3_FETHIEN2		((uint)0x10000000)      /* 0 == enable*/
#define BCSR3_FETH2_RST		((uint)0x80000000)      /* 0 == reset */

#endif /* __MACH_ADS8260_DEFS */
#endif /* __KERNEL__ */
