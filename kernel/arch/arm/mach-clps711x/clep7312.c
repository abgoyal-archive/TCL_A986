
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "common.h"

static void __init
fixup_clep7312(struct machine_desc *desc, struct tag *tags,
	    char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=1;
	mi->bank[0].start = 0xc0000000;
	mi->bank[0].size = 0x01000000;
	mi->bank[0].node = 0;
}


MACHINE_START(CLEP7212, "Cirrus Logic 7212/7312")
	/* Maintainer: Nobody */
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xff000000) >> 18) & 0xfffc,
	.boot_params	= 0xc0000100,
	.fixup		= fixup_clep7312,
	.map_io		= clps711x_map_io,
	.init_irq	= clps711x_init_irq,
	.timer		= &clps711x_timer,
MACHINE_END

