
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/init.h>

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/mach/map.h>
#include <asm/hardware/clps7111.h>

static struct map_desc clps711x_io_desc[] __initdata = {
	{
		.virtual	= CLPS7111_VIRT_BASE,
		.pfn		= __phys_to_pfn(CLPS7111_PHYS_BASE),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	}
};

void __init clps711x_map_io(void)
{
	iotable_init(clps711x_io_desc, ARRAY_SIZE(clps711x_io_desc));
}
