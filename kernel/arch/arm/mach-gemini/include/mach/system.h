
#ifndef __MACH_SYSTEM_H
#define __MACH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/global_reg.h>

static inline void arch_idle(void)
{
	/*
	 * Because of broken hardware we have to enable interrupts or the CPU
	 * will never wakeup... Acctualy it is not very good to enable
	 * interrupts here since scheduler can miss a tick, but there is
	 * no other way around this. Platforms that needs it for power saving
	 * should call enable_hlt() in init code, since by default it is
	 * disabled.
	 */
	local_irq_enable();
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	__raw_writel(RESET_GLOBAL | RESET_CPU1,
		     IO_ADDRESS(GEMINI_GLOBAL_BASE) + GLOBAL_RESET);
}

#endif /* __MACH_SYSTEM_H */
