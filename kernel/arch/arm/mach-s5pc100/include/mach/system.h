

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

#include <linux/io.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

static void arch_idle(void)
{
	/* nothing here yet */
}

static void arch_reset(char mode, const char *cmd)
{
	__raw_writel(S5PC100_SWRESET_RESETVAL, S5PC100_SWRESET);
	return;
}
#endif /* __ASM_ARCH_IRQ_H */
