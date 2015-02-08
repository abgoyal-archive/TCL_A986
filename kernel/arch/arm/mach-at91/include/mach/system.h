

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/hardware.h>
#include <mach/at91_st.h>
#include <mach/at91_dbgu.h>
#include <mach/at91_pmc.h>

static inline void arch_idle(void)
{
#ifndef CONFIG_DEBUG_KERNEL
	/*
	 * Disable the processor clock.  The processor will be automatically
	 * re-enabled by an interrupt or by a reset.
	 */
	at91_sys_write(AT91_PMC_SCDR, AT91_PMC_PCK);
#else
	/*
	 * Set the processor (CP15) into 'Wait for Interrupt' mode.
	 * Unlike disabling the processor clock via the PMC (above)
	 *  this allows the processor to be woken via JTAG.
	 */
	cpu_do_idle();
#endif
}

void (*at91_arch_reset)(void);

static inline void arch_reset(char mode, const char *cmd)
{
	/* call the CPU-specific reset function */
	if (at91_arch_reset)
		(at91_arch_reset)();
}

#endif
