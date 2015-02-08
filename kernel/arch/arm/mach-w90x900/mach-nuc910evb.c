

#include <linux/platform_device.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>

#include "nuc910.h"

static void __init nuc910evb_map_io(void)
{
	nuc910_map_io();
	nuc910_init_clocks();
}

static void __init nuc910evb_init(void)
{
	nuc910_board_init();
}

MACHINE_START(W90P910EVB, "W90P910EVB")
	/* Maintainer: Wan ZongShun */
	.phys_io	= W90X900_PA_UART,
	.io_pg_offst	= (((u32)W90X900_VA_UART) >> 18) & 0xfffc,
	.boot_params	= 0,
	.map_io		= nuc910evb_map_io,
	.init_irq	= nuc900_init_irq,
	.init_machine	= nuc910evb_init,
	.timer		= &nuc900_timer,
MACHINE_END
