

#include <linux/mm.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

/* MX21 memory map definition */
static struct map_desc imx21_io_desc[] __initdata = {
	/*
	 * this fixed mapping covers:
	 * - AIPI1
	 * - AIPI2
	 * - AITC
	 * - ROM Patch
	 * - and some reserved space
	 */
	{
		.virtual = MX21_AIPI_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX21_AIPI_BASE_ADDR),
		.length = MX21_AIPI_SIZE,
		.type = MT_DEVICE
	},
	/*
	 * this fixed mapping covers:
	 * - CSI
	 * - ATA
	 */
	{
		.virtual = MX21_SAHB1_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX21_SAHB1_BASE_ADDR),
		.length = MX21_SAHB1_SIZE,
		.type = MT_DEVICE
	},
	/*
	 * this fixed mapping covers:
	 * - EMI
	 */
	{
		.virtual = MX21_X_MEMC_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(MX21_X_MEMC_BASE_ADDR),
		.length = MX21_X_MEMC_SIZE,
		.type = MT_DEVICE
	},
};

void __init mx21_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX21);
	mxc_arch_reset_init(MX21_IO_ADDRESS(MX21_WDOG_BASE_ADDR));

	iotable_init(imx21_io_desc, ARRAY_SIZE(imx21_io_desc));
}

void __init mx21_init_irq(void)
{
	mxc_init_irq(MX21_IO_ADDRESS(MX21_AVIC_BASE_ADDR));
}
