

#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/max7301.h>
#include <linux/leds.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pxa27x.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pcm027.h>
#include "generic.h"


static unsigned long pcm027_pin_config[] __initdata = {
	/* Chip Selects */
	GPIO20_nSDCS_2,
	GPIO21_nSDCS_3,
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO80_nCS_4,
	GPIO33_nCS_5,	/* Ethernet */

	/* I2C */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,

	/* GPIO */
	GPIO52_GPIO,	/* IRQ from network controller */
#ifdef CONFIG_LEDS_GPIO
	GPIO90_GPIO,	/* PCM027_LED_CPU */
	GPIO91_GPIO,	/* PCM027_LED_HEART_BEAT */
#endif
	GPIO114_GPIO,	/* IRQ from CAN controller */
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= PCM027_ETH_PHYS + 0x300,
		.end	= PCM027_ETH_PHYS + PCM027_ETH_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= PCM027_ETH_IRQ,
		.end	= PCM027_ETH_IRQ,
		/* note: smc91x's driver doesn't use the trigger bits yet */
		.flags	= IORESOURCE_IRQ | PCM027_ETH_IRQ_EDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
};

static struct max7301_platform_data max7301_info = {
	.base = -1,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "max7301",
		.platform_data	= &max7301_info,
		.max_speed_hz	= 13000000,
		.bus_num	= 1,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static struct physmap_flash_data pcm027_flash_data = {
	.width  = 4,
};

static struct resource pcm027_flash_resource = {
	.start          = PCM027_FLASH_PHYS,
	.end            = PCM027_FLASH_PHYS + PCM027_FLASH_SIZE - 1 ,
	.flags          = IORESOURCE_MEM,
};

static struct platform_device pcm027_flash = {
	.name           = "physmap-flash",
	.id             = 0,
	.dev            = {
		.platform_data  = &pcm027_flash_data,
	},
	.resource       = &pcm027_flash_resource,
	.num_resources  = 1,
};

#ifdef CONFIG_LEDS_GPIO

static struct gpio_led pcm027_led[] = {
	{
		.name = "led0:red",	/* FIXME */
		.gpio = PCM027_LED_CPU
	},
	{
		.name = "led1:green",	/* FIXME */
		.gpio = PCM027_LED_HEARD_BEAT
	},
};

static struct gpio_led_platform_data pcm027_led_data = {
	.num_leds	= ARRAY_SIZE(pcm027_led),
	.leds		= pcm027_led
};

static struct platform_device pcm027_led_dev = {
	.name		= "leds-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &pcm027_led_data,
	},
};

#endif /* CONFIG_LEDS_GPIO */

static struct platform_device *devices[] __initdata = {
	&smc91x_device,
	&pcm027_flash,
#ifdef CONFIG_LEDS_GPIO
	&pcm027_led_dev
#endif
};

static void __init pcm027_init(void)
{
	/* system bus arbiter setting
	 * - Core_Park
	 * - LCD_wt:DMA_wt:CORE_Wt = 2:3:4
	 */
	ARB_CNTRL = ARB_CORE_PARK | 0x234;

	pxa2xx_mfp_config(pcm027_pin_config, ARRAY_SIZE(pcm027_pin_config));

	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* at last call the baseboard to initialize itself */
#ifdef CONFIG_MACH_PCM990_BASEBOARD
	pcm990_baseboard_init();
#endif

	pxa2xx_set_spi_info(1, &pxa_ssp_master_info);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init pcm027_map_io(void)
{
	pxa_map_io();

	/* initialize sleep mode regs (wake-up sources, etc) */
	PGSR0 = 0x01308000;
	PGSR1 = 0x00CF0002;
	PGSR2 = 0x0E294000;
	PGSR3 = 0x0000C000;
	PWER  = 0x40000000 | PWER_GPIO0 | PWER_GPIO1;
	PRER  = 0x00000000;
	PFER  = 0x00000003;
}

MACHINE_START(PCM027, "Phytec Messtechnik GmbH phyCORE-PXA270")
	/* Maintainer: Pengutronix */
	.boot_params	= 0xa0000100,
	.phys_io	= 0x40000000,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pcm027_map_io,
	.init_irq	= pxa27x_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= pcm027_init,
MACHINE_END
