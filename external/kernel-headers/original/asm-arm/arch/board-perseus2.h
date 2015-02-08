#ifndef __ASM_ARCH_OMAP_PERSEUS2_H
#define __ASM_ARCH_OMAP_PERSEUS2_H

#include <asm/arch/fpga.h>

#ifndef OMAP_SDRAM_DEVICE
#define OMAP_SDRAM_DEVICE		D256M_1X16_4B
#endif

#define MAXIRQNUM		IH_BOARD_BASE
#define MAXFIQNUM		MAXIRQNUM
#define MAXSWINUM		MAXIRQNUM

#define NR_IRQS			(MAXIRQNUM + 1)

#endif
