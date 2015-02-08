

#include <linux/mm.h>
#include <linux/init.h>
#include <linux/err.h>

#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-v3.h>


static struct map_desc mxc_io_desc[] __initdata = {
	{
		.virtual	= X_MEMC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(X_MEMC_BASE_ADDR),
		.length		= X_MEMC_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= AVIC_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(AVIC_BASE_ADDR),
		.length		= AVIC_SIZE,
		.type		= MT_DEVICE_NONSHARED
	}, {
		.virtual	= AIPS1_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(AIPS1_BASE_ADDR),
		.length		= AIPS1_SIZE,
		.type		= MT_DEVICE_NONSHARED
	}, {
		.virtual	= AIPS2_BASE_ADDR_VIRT,
		.pfn		= __phys_to_pfn(AIPS2_BASE_ADDR),
		.length		= AIPS2_SIZE,
		.type		= MT_DEVICE_NONSHARED
	}, {
		.virtual = SPBA0_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
		.length = SPBA0_SIZE,
		.type = MT_DEVICE_NONSHARED
	},
};

void __init mx31_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX31);
	mxc_arch_reset_init(IO_ADDRESS(WDOG_BASE_ADDR));

	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}

#ifdef CONFIG_ARCH_MX35
void __init mx35_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX35);
	mxc_iomux_v3_init(IO_ADDRESS(IOMUXC_BASE_ADDR));
	mxc_arch_reset_init(IO_ADDRESS(WDOG_BASE_ADDR));

	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}
#endif

void __init mx31_init_irq(void)
{
	mxc_init_irq(IO_ADDRESS(AVIC_BASE_ADDR));
}

void __init mx35_init_irq(void)
{
	mx31_init_irq();
}

#ifdef CONFIG_CACHE_L2X0
static int mxc_init_l2x0(void)
{
	void __iomem *l2x0_base;

	l2x0_base = ioremap(L2CC_BASE_ADDR, 4096);
	if (IS_ERR(l2x0_base)) {
		printk(KERN_ERR "remapping L2 cache area failed with %ld\n",
				PTR_ERR(l2x0_base));
		return 0;
	}

	l2x0_init(l2x0_base, 0x00030024, 0x00000000);

	return 0;
}

arch_initcall(mxc_init_l2x0);
#endif

