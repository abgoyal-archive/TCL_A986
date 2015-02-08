

#include <linux/mm.h>
#include <linux/init.h>
#include <asm/mach/map.h>
#include <asm/hardware/iop3xx.h>

static struct map_desc iop3xx_std_desc[] __initdata = {
	 {	/* mem mapped registers */
		.virtual	= IOP3XX_PERIPHERAL_VIRT_BASE,
		.pfn		= __phys_to_pfn(IOP3XX_PERIPHERAL_PHYS_BASE),
		.length		= IOP3XX_PERIPHERAL_SIZE,
		.type		= MT_UNCACHED,
	 }, {	/* PCI IO space */
		.virtual	= IOP3XX_PCI_LOWER_IO_VA,
		.pfn		= __phys_to_pfn(IOP3XX_PCI_LOWER_IO_PA),
		.length		= IOP3XX_PCI_IO_WINDOW_SIZE,
		.type		= MT_DEVICE,
	 },
};

void __init iop3xx_map_io(void)
{
	iotable_init(iop3xx_std_desc, ARRAY_SIZE(iop3xx_std_desc));
}
