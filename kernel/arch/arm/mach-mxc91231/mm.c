

#include <linux/mm.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

static struct map_desc mxc_io_desc[] __initdata = {
	{
		.virtual	= MXC91231_L2CC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_L2CC_BASE_ADDR),
		.length		= MXC91231_L2CC_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_X_MEMC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_X_MEMC_BASE_ADDR),
		.length		= MXC91231_X_MEMC_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_ROMP_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_ROMP_BASE_ADDR),
		.length		= MXC91231_ROMP_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_AVIC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_AVIC_BASE_ADDR),
		.length		= MXC91231_AVIC_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_AIPS1_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_AIPS1_BASE_ADDR),
		.length		= MXC91231_AIPS1_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_SPBA0_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_SPBA0_BASE_ADDR),
		.length		= MXC91231_SPBA0_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_SPBA1_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_SPBA1_BASE_ADDR),
		.length		= MXC91231_SPBA1_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= MXC91231_AIPS2_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(MXC91231_AIPS2_BASE_ADDR),
		.length		= MXC91231_AIPS2_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init mxc91231_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MXC91231);

	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}

void __init mxc91231_init_irq(void)
{
	mxc_init_irq(MXC91231_IO_ADDRESS(MXC91231_AVIC_BASE_ADDR));
}
