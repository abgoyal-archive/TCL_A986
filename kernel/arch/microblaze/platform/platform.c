

#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/prom.h>
#include <asm/setup.h>

static struct of_device_id xilinx_of_bus_ids[] __initdata = {
	{ .compatible = "simple-bus", },
	{ .compatible = "xlnx,plb-v46-1.00.a", },
	{ .compatible = "xlnx,opb-v20-1.10.c", },
	{ .compatible = "xlnx,opb-v20-1.10.b", },
	{ .compatible = "xlnx,compound", },
	{}
};

static int __init microblaze_device_probe(void)
{
	of_platform_bus_probe(NULL, xilinx_of_bus_ids, NULL);
	of_platform_reset_gpio_probe();
	return 0;
}
device_initcall(microblaze_device_probe);
