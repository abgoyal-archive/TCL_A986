

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include <mach/hardware.h>

static void arch_idle(void)
{
	CPU_REG (PMU_BASE, PMU_MODE) = PMU_MODE_IDLE;
	nop();
	nop();
	CPU_REG (PMU_BASE, PMU_MODE) = PMU_MODE_RUN;
	nop();
	nop();
}


static __inline__ void arch_reset(char mode, const char *cmd)
{
	CPU_REG (PMU_BASE, PMU_STAT) |= PMU_WARMRESET;
}

#endif
