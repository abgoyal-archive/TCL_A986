
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include "netx-regs.h"

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	writel(NETX_SYSTEM_RES_CR_FIRMW_RES_EN | NETX_SYSTEM_RES_CR_FIRMW_RES,
	       NETX_SYSTEM_RES_CR);
}

#endif

