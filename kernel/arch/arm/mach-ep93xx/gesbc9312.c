

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>


static struct physmap_flash_data gesbc9312_flash_data = {
	.width		= 4,
};

static struct resource gesbc9312_flash_resource = {
	.start		= EP93XX_CS6_PHYS_BASE,
	.end		= EP93XX_CS6_PHYS_BASE + SZ_8M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device gesbc9312_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &gesbc9312_flash_data,
	},
	.num_resources	= 1,
	.resource	= &gesbc9312_flash_resource,
};

static struct ep93xx_eth_data __initdata gesbc9312_eth_data = {
	.phy_id		= 1,
};

static void __init gesbc9312_init_machine(void)
{
	ep93xx_init_devices();
	platform_device_register(&gesbc9312_flash);

	ep93xx_register_eth(&gesbc9312_eth_data, 0);
}

MACHINE_START(GESBC9312, "Glomation GESBC-9312-sx")
	/* Maintainer: Lennert Buytenhek <buytenh@wantstofly.org> */
	.phys_io	= EP93XX_APB_PHYS_BASE,
	.io_pg_offst	= ((EP93XX_APB_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= EP93XX_SDCE3_PHYS_BASE_SYNC + 0x100,
	.map_io		= ep93xx_map_io,
	.init_irq	= ep93xx_init_irq,
	.timer		= &ep93xx_timer,
	.init_machine	= gesbc9312_init_machine,
MACHINE_END
