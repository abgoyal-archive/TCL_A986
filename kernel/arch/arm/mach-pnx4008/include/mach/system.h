
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>

static void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	cpu_reset(0);
}

#endif
