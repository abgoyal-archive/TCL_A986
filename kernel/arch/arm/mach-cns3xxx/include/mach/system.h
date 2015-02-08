

#ifndef __MACH_SYSTEM_H
#define __MACH_SYSTEM_H

#include <linux/io.h>
#include <asm/proc-fns.h>
#include <mach/hardware.h>

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

void arch_reset(char mode, const char *cmd);

#endif
