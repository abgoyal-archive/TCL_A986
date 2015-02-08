
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/hardware.h>

static inline void arch_idle(void)
{
	*(unsigned long *)(IO_BASE + 0x50004) = 1;	/* idle mode */
}

static inline void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		cpu_reset(0);
	}
}

#endif
