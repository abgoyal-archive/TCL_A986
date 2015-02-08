
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/common.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	if (davinci_soc_info.reset)
		davinci_soc_info.reset(davinci_soc_info.reset_device);
}

#endif /* __ASM_ARCH_SYSTEM_H */
