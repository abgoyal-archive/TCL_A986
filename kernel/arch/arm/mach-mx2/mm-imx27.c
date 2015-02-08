

#include <linux/mm.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

/* MX27 memory map definition */
static struct map_desc imx27_io_desc[] __initdata = {
	/*
	 * this fixed mapping covers:
	 * - AIPI1
	 * - AIPI2
	 * - AITC
	 * - ROM Patch
	 * - and some reserved space
	 */
	{
		.virtual = MX27_AIPI_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX27_AIPI_BASE_ADDR),
		.length = MX27_AIPI_SIZE,
		.type = MT_DEVICE
	},
	/*
	 * this fixed mapping covers:
	 * - CSI
	 * - ATA
	 */
	{
		.virtual = MX27_SAHB1_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX27_SAHB1_BASE_ADDR),
		.length = MX27_SAHB1_SIZE,
		.type = MT_DEVICE
	},
	/*
	 * this fixed mapping covers:
	 * - EMI
	 */
	{
		.virtual = MX27_X_MEMC_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX27_X_MEMC_BASE_ADDR),
		.length = MX27_X_MEMC_SIZE,
		.type = MT_DEVICE
	},
};

void __init mx27_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX27);
	mxc_arch_reset_init(MX27_IO_ADDRESS(MX27_WDOG_BASE_ADDR));

	iotable_init(imx27_io_desc, ARRAY_SIZE(imx27_io_desc));
}

void __init mx27_init_irq(void)
{
	mxc_init_irq(MX27_IO_ADDRESS(MX27_AVIC_BASE_ADDR));
}
