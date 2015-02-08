

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/device.h>

#include <asm/setup.h>
#include <asm/types.h>
#include <asm/mach-types.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/mach/arch.h>
#include <mach/hardware.h>
#include "common.h"

MACHINE_START(H7201, "Hynix GMS30C7201")
	/* Maintainer: Robert Schwebel, Pengutronix */
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0xc0001000,
	.map_io		= h720x_map_io,
	.init_irq	= h720x_init_irq,
	.timer		= &h7201_timer,
MACHINE_END
