

#ifndef __ASM_ARCH_MXC_SYSTEM_H__
#define __ASM_ARCH_MXC_SYSTEM_H__

#include <mach/hardware.h>
#include <mach/common.h>

static inline void arch_idle(void)
{
#ifdef CONFIG_ARCH_MXC91231
	if (cpu_is_mxc91231()) {
		/* Need this to set DSM low-power mode */
		mxc91231_prepare_idle();
	}
#endif

	cpu_do_idle();
}

void arch_reset(char mode, const char *cmd);

#endif /* __ASM_ARCH_MXC_SYSTEM_H__ */
