
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <asm/mach/map.h>

#include <mach/common.h>
#include <mach/hardware.h>

static struct map_desc imx_io_desc[] __initdata = {
	{
		.virtual	= IMX_IO_BASE,
		.pfn		= __phys_to_pfn(IMX_IO_PHYS),
		.length		= IMX_IO_SIZE,
		.type		= MT_DEVICE
	}
};

void __init mx1_map_io(void)
{
	mxc_set_cpu_type(MXC_CPU_MX1);
	mxc_arch_reset_init(IO_ADDRESS(WDT_BASE_ADDR));

	iotable_init(imx_io_desc, ARRAY_SIZE(imx_io_desc));
}

void __init mx1_init_irq(void)
{
	mxc_init_irq(IO_ADDRESS(AVIC_BASE_ADDR));
}

