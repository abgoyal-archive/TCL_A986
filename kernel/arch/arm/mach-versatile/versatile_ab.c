

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>

#include "core.h"

MACHINE_START(VERSATILE_AB, "ARM-Versatile AB")
	/* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
	.phys_io	= 0x101f1000,
	.io_pg_offst	= ((0xf11f1000) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= versatile_map_io,
	.init_irq	= versatile_init_irq,
	.timer		= &versatile_timer,
	.init_machine	= versatile_init,
MACHINE_END
