
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/sizes.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <mach/autcpu12.h>

#include "common.h"


static struct map_desc autcpu12_io_desc[] __initdata = {
	/* memory-mapped extra io and CS8900A Ethernet chip */
 	/* ethernet chip */
 	{
		.virtual	= AUTCPU12_VIRT_CS8900A,
		.pfn		= __phys_to_pfn(AUTCPU12_PHYS_CS8900A),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	}
};

void __init autcpu12_map_io(void)
{
        clps711x_map_io();
        iotable_init(autcpu12_io_desc, ARRAY_SIZE(autcpu12_io_desc));
}

MACHINE_START(AUTCPU12, "autronix autcpu12")
	/* Maintainer: Thomas Gleixner */
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xff000000) >> 18) & 0xfffc,
	.boot_params	= 0xc0020000,
	.map_io		= autcpu12_map_io,
	.init_irq	= clps711x_init_irq,
	.timer		= &clps711x_timer,
MACHINE_END

