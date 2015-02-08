
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/platform.h>

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
	u32 val;

	val = __raw_readl(IO_ADDRESS(VERSATILE_SYS_RESETCTL)) & ~0x7;
	val |= 0x105;

	__raw_writel(0xa05f, IO_ADDRESS(VERSATILE_SYS_LOCK));
	__raw_writel(val, IO_ADDRESS(VERSATILE_SYS_RESETCTL));
	__raw_writel(0, IO_ADDRESS(VERSATILE_SYS_LOCK));
}

#endif
