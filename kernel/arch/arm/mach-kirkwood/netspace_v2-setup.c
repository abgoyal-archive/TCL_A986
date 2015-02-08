

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/ata_platform.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/kirkwood.h>
#include <plat/time.h>
#include "common.h"
#include "mpp.h"


static struct mtd_partition netspace_v2_flash_parts[] = {
	{
		.name = "u-boot",
		.size = MTDPART_SIZ_FULL,
		.offset = 0,
		.mask_flags = MTD_WRITEABLE, /* force read-only */
	},
};

static const struct flash_platform_data netspace_v2_flash = {
	.type		= "mx25l4005a",
	.name		= "spi_flash",
	.parts		= netspace_v2_flash_parts,
	.nr_parts	= ARRAY_SIZE(netspace_v2_flash_parts),
};

static struct spi_board_info __initdata netspace_v2_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &netspace_v2_flash,
		.irq		= -1,
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};


static struct mv643xx_eth_platform_data netspace_v2_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};


static struct at24_platform_data at24c04 = {
	.byte_len	= SZ_4K / 8,
	.page_size	= 16,
};


static struct i2c_board_info __initdata netspace_v2_i2c_info[] = {
	{
		I2C_BOARD_INFO("24c04", 0x50),
		.platform_data  = &at24c04,
	}
};


static struct mv_sata_platform_data netspace_v2_sata_data = {
	.n_ports	= 2,
};

#define NETSPACE_V2_GPIO_SATA0_POWER	16
#define NETSPACE_V2_GPIO_SATA1_POWER	17

static void __init netspace_v2_sata_power_init(void)
{
	int err;

	err = gpio_request(NETSPACE_V2_GPIO_SATA0_POWER, "SATA0 power");
	if (err == 0) {
		err = gpio_direction_output(NETSPACE_V2_GPIO_SATA0_POWER, 1);
		if (err)
			gpio_free(NETSPACE_V2_GPIO_SATA0_POWER);
	}
	if (err)
		pr_err("netspace_v2: failed to setup SATA0 power\n");
}


#define NETSPACE_V2_PUSH_BUTTON		32

static struct gpio_keys_button netspace_v2_buttons[] = {
	[0] = {
		.code		= KEY_POWER,
		.gpio		= NETSPACE_V2_PUSH_BUTTON,
		.desc		= "Power push button",
		.active_low	= 0,
	},
};

static struct gpio_keys_platform_data netspace_v2_button_data = {
	.buttons	= netspace_v2_buttons,
	.nbuttons	= ARRAY_SIZE(netspace_v2_buttons),
};

static struct platform_device netspace_v2_gpio_buttons = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data 	= &netspace_v2_button_data,
	},
};



#define NETSPACE_V2_GPIO_RED_LED	12
#define NETSPACE_V2_GPIO_BLUE_LED_SLOW	29
#define NETSPACE_V2_GPIO_BLUE_LED_CMD	30


static struct gpio_led netspace_v2_gpio_led_pins[] = {
	{
		.name			= "ns_v2:blue:sata",
		.default_trigger	= "default-on",
		.gpio			= NETSPACE_V2_GPIO_BLUE_LED_CMD,
		.active_low		= 1,
	},
	{
		.name			= "ns_v2:red:fail",
		.gpio			= NETSPACE_V2_GPIO_RED_LED,
	},
};

static struct gpio_led_platform_data netspace_v2_gpio_leds_data = {
	.num_leds	= ARRAY_SIZE(netspace_v2_gpio_led_pins),
	.leds		= netspace_v2_gpio_led_pins,
};

static struct platform_device netspace_v2_gpio_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &netspace_v2_gpio_leds_data,
	},
};

static void __init netspace_v2_gpio_leds_init(void)
{
	int err;

	/* Configure register slow_led to allow SATA activity LED blinking */
	err = gpio_request(NETSPACE_V2_GPIO_BLUE_LED_SLOW, "blue LED slow");
	if (err == 0) {
		err = gpio_direction_output(NETSPACE_V2_GPIO_BLUE_LED_SLOW, 0);
		if (err)
			gpio_free(NETSPACE_V2_GPIO_BLUE_LED_SLOW);
	}
	if (err)
		pr_err("netspace_v2: failed to configure blue LED slow GPIO\n");

	platform_device_register(&netspace_v2_gpio_leds);
}


static void netspace_v2_timer_init(void)
{
	kirkwood_tclk = 166666667;
	orion_time_init(IRQ_KIRKWOOD_BRIDGE, kirkwood_tclk);
}

struct sys_timer netspace_v2_timer = {
	.init = netspace_v2_timer_init,
};


static unsigned int netspace_v2_mpp_config[] __initdata = {
	MPP0_SPI_SCn,
	MPP1_SPI_MOSI,
	MPP2_SPI_SCK,
	MPP3_SPI_MISO,
	MPP4_NF_IO6,
	MPP5_NF_IO7,
	MPP6_SYSRST_OUTn,
	MPP8_TW_SDA,
	MPP9_TW_SCK,
	MPP10_UART0_TXD,
	MPP11_UART0_RXD,
	MPP12_GPO,		/* Red led */
	MPP14_GPIO,		/* USB fuse */
	MPP16_GPIO,		/* SATA 0 power */
	MPP18_NF_IO0,
	MPP19_NF_IO1,
	MPP20_SATA1_ACTn,
	MPP21_SATA0_ACTn,
	MPP24_GPIO,		/* USB mode select */
	MPP25_GPIO,		/* Fan rotation fail */
	MPP26_GPIO,		/* USB device vbus */
	MPP28_GPIO,		/* USB enable host vbus */
	MPP29_GPIO,		/* Blue led (slow register) */
	MPP30_GPIO,		/* Blue led (command register) */
	MPP31_GPIO,		/* Board power off */
	MPP32_GPIO, 		/* Power button (0 = Released, 1 = Pushed) */
	0
};

#define NETSPACE_V2_GPIO_POWER_OFF	31

static void netspace_v2_power_off(void)
{
	gpio_set_value(NETSPACE_V2_GPIO_POWER_OFF, 1);
}

static void __init netspace_v2_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();
	kirkwood_mpp_conf(netspace_v2_mpp_config);

	netspace_v2_sata_power_init();

	kirkwood_ehci_init();
	kirkwood_ge00_init(&netspace_v2_ge00_data);
	kirkwood_sata_init(&netspace_v2_sata_data);
	kirkwood_uart0_init();
	spi_register_board_info(netspace_v2_spi_slave_info,
				ARRAY_SIZE(netspace_v2_spi_slave_info));
	kirkwood_spi_init();
	kirkwood_i2c_init();
	i2c_register_board_info(0, netspace_v2_i2c_info,
				ARRAY_SIZE(netspace_v2_i2c_info));

	netspace_v2_gpio_leds_init();
	platform_device_register(&netspace_v2_gpio_buttons);

	if (gpio_request(NETSPACE_V2_GPIO_POWER_OFF, "power-off") == 0 &&
	    gpio_direction_output(NETSPACE_V2_GPIO_POWER_OFF, 0) == 0)
		pm_power_off = netspace_v2_power_off;
	else
		pr_err("netspace_v2: failed to configure power-off GPIO\n");
}

#ifdef CONFIG_MACH_NETSPACE_V2
MACHINE_START(NETSPACE_V2, "LaCie Network Space v2")
	.phys_io	= KIRKWOOD_REGS_PHYS_BASE,
	.io_pg_offst	= ((KIRKWOOD_REGS_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.init_machine	= netspace_v2_init,
	.map_io		= kirkwood_map_io,
	.init_irq	= kirkwood_init_irq,
	.timer		= &netspace_v2_timer,
MACHINE_END
#endif

#ifdef CONFIG_MACH_INETSPACE_V2
MACHINE_START(INETSPACE_V2, "LaCie Internet Space v2")
	.phys_io	= KIRKWOOD_REGS_PHYS_BASE,
	.io_pg_offst	= ((KIRKWOOD_REGS_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.init_machine	= netspace_v2_init,
	.map_io		= kirkwood_map_io,
	.init_irq	= kirkwood_init_irq,
	.timer		= &netspace_v2_timer,
MACHINE_END
#endif
