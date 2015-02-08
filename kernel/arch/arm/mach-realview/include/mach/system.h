
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>

void (*realview_reset)(char mode);

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	/*
	 * To reset, we hit the on-board reset register
	 * in the system FPGA
	 */
	if (realview_reset)
		realview_reset(mode);
}

#endif
