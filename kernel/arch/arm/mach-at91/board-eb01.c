

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/board.h>
#include "generic.h"

static void __init at91eb01_map_io(void)
{
	at91x40_initialize(40000000);
}

MACHINE_START(AT91EB01, "Atmel AT91 EB01")
	/* Maintainer: Greg Ungerer <gerg@snapgear.com> */
	.timer		= &at91x40_timer,
	.init_irq	= at91x40_init_interrupts,
	.map_io		= at91eb01_map_io,
MACHINE_END

