
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/cm.h>

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
	cm_control(CM_CTRL_RESET, CM_CTRL_RESET);
}

#endif
