

#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/page.h>
#include <asm/mach/map.h>
#include <mach/addr-map.h>

#include "common.h"

static struct map_desc standard_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(APB_PHYS_BASE),
		.virtual	= APB_VIRT_BASE,
		.length		= APB_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(AXI_PHYS_BASE),
		.virtual	= AXI_VIRT_BASE,
		.length		= AXI_PHYS_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
}
