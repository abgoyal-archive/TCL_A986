

#include <mach/hardware.h>
#include <plat/watchdog-reset.h>

extern void (*s3c24xx_reset_hook)(void);

static void
arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		cpu_reset(0);
	}

	if (s3c24xx_reset_hook)
		s3c24xx_reset_hook();

	arch_wdt_reset();

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}
