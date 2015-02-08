

#ifndef _ASM_AX88796_H
#define _ASM_AX88796_H

#include <asm/mb-regs.h>

#define AX88796_IOADDR		(__region_CS1 + 0x200)
#define AX88796_IRQ		IRQ_CPU_EXTERNAL7
#define AX88796_FULL_DUPLEX	0			/* force full duplex */
#define AX88796_BUS_INFO	"CS1#+0x200"		/* bus info for ethtool */

#endif /* _ASM_AX88796_H */
