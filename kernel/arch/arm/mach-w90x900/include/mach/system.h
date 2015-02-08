

#include <linux/io.h>
#include <asm/proc-fns.h>
#include <mach/map.h>
#include <mach/regs-timer.h>

#define	WTCR	(TMR_BA + 0x1C)
#define	WTCLK	(1 << 10)
#define	WTE	(1 << 7)
#define	WTRE	(1 << 1)

static void arch_idle(void)
{
}

static void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		/* Jump into ROM at address 0 */
		cpu_reset(0);
	} else {
		__raw_writel(WTE | WTRE | WTCLK, WTCR);
	}
}

