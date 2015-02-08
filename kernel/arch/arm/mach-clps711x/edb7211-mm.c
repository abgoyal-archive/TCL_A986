
#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/hardware.h>
#include <asm/page.h>
#include <asm/sizes.h>
 
#include <asm/mach/map.h>

extern void clps711x_map_io(void);

static struct map_desc edb7211_io_desc[] __initdata = {
 	{	/* memory-mapped extra keyboard row */
	 	.virtual 	= EP7211_VIRT_EXTKBD,
		.pfn		= __phys_to_pfn(EP7211_PHYS_EXTKBD),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	}, {	/* and CS8900A Ethernet chip */
		.virtual	= EP7211_VIRT_CS8900A,
		.pfn		= __phys_to_pfn(EP7211_PHYS_CS8900A),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	}, { 	/* flash banks */
		.virtual	= EP7211_VIRT_FLASH1,
		.pfn		= __phys_to_pfn(EP7211_PHYS_FLASH1),
		.length		= SZ_8M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= EP7211_VIRT_FLASH2,
		.pfn		= __phys_to_pfn(EP7211_PHYS_FLASH2),
		.length		= SZ_8M,
		.type		= MT_DEVICE,
	}
};

void __init edb7211_map_io(void)
{
        clps711x_map_io();
        iotable_init(edb7211_io_desc, ARRAY_SIZE(edb7211_io_desc));
}

