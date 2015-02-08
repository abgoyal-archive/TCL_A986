

#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/mach-au1x00/au1000.h>

static struct resource xxs1500_pcmcia_res[] = {
	{
		.name	= "pcmcia-io",
		.flags	= IORESOURCE_MEM,
		.start	= PCMCIA_IO_PHYS_ADDR,
		.end	= PCMCIA_IO_PHYS_ADDR + 0x000400000 - 1,
	},
	{
		.name	= "pcmcia-attr",
		.flags	= IORESOURCE_MEM,
		.start	= PCMCIA_ATTR_PHYS_ADDR,
		.end	= PCMCIA_ATTR_PHYS_ADDR + 0x000400000 - 1,
	},
	{
		.name	= "pcmcia-mem",
		.flags	= IORESOURCE_MEM,
		.start	= PCMCIA_MEM_PHYS_ADDR,
		.end	= PCMCIA_MEM_PHYS_ADDR + 0x000400000 - 1,
	},
};

static struct platform_device xxs1500_pcmcia_dev = {
	.name		= "xxs1500_pcmcia",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(xxs1500_pcmcia_res),
	.resource	= xxs1500_pcmcia_res,
};

static struct platform_device *xxs1500_devs[] __initdata = {
	&xxs1500_pcmcia_dev,
};

static int __init xxs1500_dev_init(void)
{
	return platform_add_devices(xxs1500_devs,
				    ARRAY_SIZE(xxs1500_devs));
}
device_initcall(xxs1500_dev_init);
