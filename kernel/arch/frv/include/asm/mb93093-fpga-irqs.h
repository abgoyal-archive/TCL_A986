

#ifndef _ASM_MB93093_FPGA_IRQS_H
#define _ASM_MB93093_FPGA_IRQS_H

#include <asm/irq.h>

#ifndef __ASSEMBLY__

/* IRQ IDs presented to drivers */
enum {
	IRQ_FPGA_PUSH_BUTTON_SW1_5		= IRQ_BASE_FPGA + 8,
	IRQ_FPGA_ROCKER_C_SW8			= IRQ_BASE_FPGA + 9,
	IRQ_FPGA_ROCKER_C_SW9			= IRQ_BASE_FPGA + 10,
};


#endif /* !__ASSEMBLY__ */

#endif /* _ASM_MB93093_FPGA_IRQS_H */
