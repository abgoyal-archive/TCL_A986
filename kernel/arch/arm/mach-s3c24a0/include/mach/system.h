

#include <mach/hardware.h>
#include <asm/io.h>

#include <mach/map.h>

static void arch_idle(void)
{
	/* currently no specific idle support. */
}

void (*s3c24xx_reset_hook)(void);

#include <asm/plat-s3c24xx/system-reset.h>
