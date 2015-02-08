
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/hardware/iomd.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	iomd_writeb(0, IOMD_ROMCR0);

	/*
	 * Jump into the ROM
	 */
	cpu_reset(0);
}
