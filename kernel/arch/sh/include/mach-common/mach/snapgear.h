

#ifndef _ASM_SH_IO_SNAPGEAR_H
#define _ASM_SH_IO_SNAPGEAR_H

#if defined(CONFIG_CPU_SH4)

#define IRL0_IRQ	2
#define IRL0_PRIORITY	13

#define IRL1_IRQ	5
#define IRL1_PRIORITY	10

#define IRL2_IRQ	8
#define IRL2_PRIORITY	7

#define IRL3_IRQ	11
#define IRL3_PRIORITY	4
#endif

#define __IO_PREFIX	snapgear
#include <asm/io_generic.h>

#ifdef CONFIG_SH_SECUREEDGE5410

#define SECUREEDGE_IOPORT_ADDR ((volatile short *) 0xb0000000)
extern unsigned short secureedge5410_ioport;

#define SECUREEDGE_WRITE_IOPORT(val, mask) (*SECUREEDGE_IOPORT_ADDR = \
	 (secureedge5410_ioport = \
			((secureedge5410_ioport & ~(mask)) | ((val) & (mask)))))
#define SECUREEDGE_READ_IOPORT() \
	 ((*SECUREEDGE_IOPORT_ADDR&0x0817) | (secureedge5410_ioport&~0x0817))
#endif

#endif /* _ASM_SH_IO_SNAPGEAR_H */
