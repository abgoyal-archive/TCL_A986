

#ifndef __ASM_DEC_MACHTYPE_H
#define __ASM_DEC_MACHTYPE_H

#include <asm/bootinfo.h>

#define TURBOCHANNEL	(mips_machtype == MACH_DS5000_200 || \
			 mips_machtype == MACH_DS5000_1XX || \
			 mips_machtype == MACH_DS5000_XX  || \
			 mips_machtype == MACH_DS5000_2X0 || \
			 mips_machtype == MACH_DS5900)

#define IOASIC		(mips_machtype == MACH_DS5000_1XX || \
			 mips_machtype == MACH_DS5000_XX  || \
			 mips_machtype == MACH_DS5000_2X0 || \
			 mips_machtype == MACH_DS5900)

#endif
