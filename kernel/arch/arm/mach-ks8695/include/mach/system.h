

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/regs-timer.h>

static void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks,
	 */
	cpu_do_idle();

}

static void arch_reset(char mode, const char *cmd)
{
	unsigned int reg;

	if (mode == 's')
		cpu_reset(0);

	/* disable timer0 */
	reg = __raw_readl(KS8695_TMR_VA + KS8695_TMCON);
	__raw_writel(reg & ~TMCON_T0EN, KS8695_TMR_VA + KS8695_TMCON);

	/* enable watchdog mode */
	__raw_writel((10 << 8) | T0TC_WATCHDOG, KS8695_TMR_VA + KS8695_T0TC);

	/* re-enable timer0 */
	__raw_writel(reg | TMCON_T0EN, KS8695_TMR_VA + KS8695_TMCON);
}

#endif
