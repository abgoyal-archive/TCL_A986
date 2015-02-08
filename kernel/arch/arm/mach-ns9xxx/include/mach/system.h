
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/proc-fns.h>
#include <mach/processor.h>
#include <mach/processor-ns9360.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_PROCESSOR_NS9360
	if (processor_is_ns9360())
		ns9360_reset(mode);
	else
#endif
		BUG();

	BUG();
}

#endif /* ifndef __ASM_ARCH_SYSTEM_H */
