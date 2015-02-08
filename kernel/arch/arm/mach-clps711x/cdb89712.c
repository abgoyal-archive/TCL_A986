
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"

static struct map_desc cdb89712_io_desc[] __initdata = {
	{
		.virtual	= ETHER_BASE,
		.pfn		=__phys_to_pfn(ETHER_START),
		.length		= ETHER_SIZE,
		.type		= MT_DEVICE
	}
};

static void __init cdb89712_map_io(void)
{
	clps711x_map_io();
	iotable_init(cdb89712_io_desc, ARRAY_SIZE(cdb89712_io_desc));
}

MACHINE_START(CDB89712, "Cirrus-CDB89712")
	/* Maintainer: Ray Lehtiniemi */
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xff000000) >> 18) & 0xfffc,
	.boot_params	= 0xc0000100,
	.map_io		= cdb89712_map_io,
	.init_irq	= clps711x_init_irq,
	.timer		= &clps711x_timer,
MACHINE_END
