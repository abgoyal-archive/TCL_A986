
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/hardware/clps7111.h>

static inline void arch_idle(void)
{
	clps_writel(1, HALT);
	__asm__ __volatile__(
	"mov	r0, r0\n\
	mov	r0, r0");
}

static inline void arch_reset(char mode, const char *cmd)
{
	cpu_reset(0);
}

#endif
