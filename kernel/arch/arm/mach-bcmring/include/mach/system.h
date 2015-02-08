
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/csp/chipcHw_inline.h>

extern int bcmring_arch_warm_reboot;

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	printk("arch_reset:%c %x\n", mode, bcmring_arch_warm_reboot);

	if (mode == 'h') {
		/* Reboot configured in proc entry */
		if (bcmring_arch_warm_reboot) {
			printk("warm reset\n");
			/* Issue Warm reset (do not reset ethernet switch, keep alive) */
			chipcHw_reset(chipcHw_REG_SOFT_RESET_CHIP_WARM);
		} else {
			/* Force reset of everything */
			printk("force reset\n");
			chipcHw_reset(chipcHw_REG_SOFT_RESET_CHIP_SOFT);
		}
	} else {
		/* Force reset of everything */
		printk("force reset\n");
		chipcHw_reset(chipcHw_REG_SOFT_RESET_CHIP_SOFT);
	}
}

#endif
