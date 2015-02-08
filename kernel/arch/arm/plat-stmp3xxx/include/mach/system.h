

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/proc-fns.h>
#include <mach/platform.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-power.h>

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
	/* Set BATTCHRG to default value */
	__raw_writel(0x00010000, REGS_POWER_BASE + HW_POWER_CHARGE);

	/* Set MINPWR to default value   */
	__raw_writel(0, REGS_POWER_BASE + HW_POWER_MINPWR);

	/* Reset digital side of chip (but not power or RTC) */
	__raw_writel(BM_CLKCTRL_RESET_DIG,
			REGS_CLKCTRL_BASE + HW_CLKCTRL_RESET);

	/* Should not return */
}

#endif
