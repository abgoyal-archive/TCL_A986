

#include <linux/mm.h>
#include <linux/init.h>
#include <linux/err.h>

#include <asm/pgtable.h>
#include <asm/mach/map.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mx25.h>
#include <mach/iomux-v3.h>

static struct map_desc mxc_io_desc[] __initdata = {
	{
		.virtual	= MX25_AVIC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MX25_AVIC_BASE_ADDR),
		.length		= MX25_AVIC_SIZE,
		.type		= MT_DEVICE_NONSHARED
	}, {
		.virtual	= MX25_AIPS1_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MX25_AIPS1_BASE_ADDR),
		.length		= MX25_AIPS1_SIZE,
		.type		= MT_DEVICE_NONSHARED
	}, {
		.virtual	= MX25_AIPS2_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MX25_AIPS2_BASE_ADDR),
		.length		= MX25_AIPS2_SIZE,
		.type		= MT_DEVICE_NONSHARED
	},
};

void __init mx25_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX25);
	mxc_iomux_v3_init(MX25_IO_ADDRESS(MX25_IOMUXC_BASE_ADDR));
	mxc_arch_reset_init(MX25_IO_ADDRESS(MX25_WDOG_BASE_ADDR));

	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}

void __init mx25_init_irq(void)
{
	mxc_init_irq((void __iomem *)MX25_AVIC_BASE_ADDR_VIRT);
}

