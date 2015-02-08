

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/bridge-regs.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	/*
	 * Enable and issue soft reset
	 */
	orion5x_setbits(RSTOUTn_MASK, (1 << 2));
	orion5x_setbits(CPU_SOFT_RESET, 1);
}


#endif
