

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


static struct mtd_partition netxbig_v2_flash_parts[] = {
	{
		.name = "u-boot",
		.size = MTDPART_SIZ_FULL,
		.offset = 0,
		.mask_flags = MTD_WRITEABLE, /* force read-only */
	},
};

static const struct flash_platform_data netxbig_v2_flash = {
	.type		= "mx25l4005a",
	.name		= "spi_flash",
	.parts		= netxbig_v2_flash_parts,
	.nr_parts	= ARRAY_SIZE(netxbig_v2_flash_parts),
};

static struct spi_board_info __initdata netxbig_v2_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &netxbig_v2_flash,
		.irq		= -1,
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
};


static struct mv643xx_eth_platform_data netxbig_v2_ge00_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

static struct mv643xx_eth_platform_data netxbig_v2_ge01_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0),
};


static struct at24_platform_data at24c04 = {
	.byte_len	= SZ_4K / 8,
	.page_size	= 16,
};


static struct i2c_board_info __initdata netxbig_v2_i2c_info[] = {
	{
		I2C_BOARD_INFO("24c04", 0x50),
		.platform_data  = &at24c04,
	}
};


static struct mv_sata_platform_data netxbig_v2_sata_data = {
	.n_ports	= 2,
};

static int __initdata netxbig_v2_gpio_hdd_power[] = { 16, 17, 41, 42, 43 };

static void __init netxbig_v2_sata_power_init(void)
{
	int i;
	int err;
	int hdd_nb;

	if (machine_is_net2big_v2())
		hdd_nb = 2;
	else
		hdd_nb = 5;

	/* Power up all hard disks. */
	for (i = 0; i < hdd_nb; i++) {
		err = gpio_request(netxbig_v2_gpio_hdd_power[i], NULL);
		if (err == 0) {
			err = gpio_direction_output(
					netxbig_v2_gpio_hdd_power[i], 1);
			/* Free the HDD power GPIOs. This allow user-space to
			 * configure them via the gpiolib sysfs interface. */
			gpio_free(netxbig_v2_gpio_hdd_power[i]);
		}
		if (err)
			pr_err("netxbig_v2: failed to power up HDD%d\n", i + 1);
	}
}


#define NETXBIG_V2_GPIO_SWITCH_POWER_ON		13
#define NETXBIG_V2_GPIO_SWITCH_POWER_OFF	15
#define NETXBIG_V2_GPIO_FUNC_BUTTON		34

#define NETXBIG_V2_SWITCH_POWER_ON		0x1
#define NETXBIG_V2_SWITCH_POWER_OFF		0x2

static struct gpio_keys_button netxbig_v2_buttons[] = {
	[0] = {
		.type           = EV_SW,
		.code           = NETXBIG_V2_SWITCH_POWER_ON,
		.gpio           = NETXBIG_V2_GPIO_SWITCH_POWER_ON,
		.desc           = "Back power switch (on|auto)",
		.active_low     = 1,
	},
	[1] = {
		.type           = EV_SW,
		.code           = NETXBIG_V2_SWITCH_POWER_OFF,
		.gpio           = NETXBIG_V2_GPIO_SWITCH_POWER_OFF,
		.desc           = "Back power switch (auto|off)",
		.active_low     = 1,
	},
	[2] = {
		.code		= KEY_OPTION,
		.gpio		= NETXBIG_V2_GPIO_FUNC_BUTTON,
		.desc		= "Function button",
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data netxbig_v2_button_data = {
	.buttons	= netxbig_v2_buttons,
	.nbuttons	= ARRAY_SIZE(netxbig_v2_buttons),
};

static struct platform_device netxbig_v2_gpio_buttons = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &netxbig_v2_button_data,
	},
};




static void netxbig_v2_timer_init(void)
{
	kirkwood_tclk = 166666667;
	orion_time_init(IRQ_KIRKWOOD_BRIDGE, kirkwood_tclk);
}

struct sys_timer netxbig_v2_timer = {
	.init = netxbig_v2_timer_init,
};


static unsigned int net2big_v2_mpp_config[] __initdata = {
	MPP0_SPI_SCn,
	MPP1_SPI_MOSI,
	MPP2_SPI_SCK,
	MPP3_SPI_MISO,
	MPP6_SYSRST_OUTn,
	MPP7_GPO,		/* Request power-off */
	MPP8_TW_SDA,
	MPP9_TW_SCK,
	MPP10_UART0_TXD,
	MPP11_UART0_RXD,
	MPP13_GPIO,		/* Rear power switch (on|auto) */
	MPP14_GPIO,		/* USB fuse alarm */
	MPP15_GPIO,		/* Rear power switch (auto|off) */
	MPP16_GPIO,		/* SATA HDD1 power */
	MPP17_GPIO,		/* SATA HDD2 power */
	MPP20_SATA1_ACTn,
	MPP21_SATA0_ACTn,
	MPP24_GPIO,		/* USB mode select */
	MPP26_GPIO,		/* USB device vbus */
	MPP28_GPIO,		/* USB enable host vbus */
	MPP29_GPIO,		/* CPLD extension ALE */
	MPP34_GPIO,		/* Rear Push button */
	MPP35_GPIO,		/* Inhibit switch power-off */
	MPP36_GPIO,		/* SATA HDD1 presence */
	MPP37_GPIO,		/* SATA HDD2 presence */
	MPP40_GPIO,		/* eSATA presence */
	MPP44_GPIO,		/* CPLD extension (data 0) */
	MPP45_GPIO,		/* CPLD extension (data 1) */
	MPP46_GPIO,		/* CPLD extension (data 2) */
	MPP47_GPIO,		/* CPLD extension (addr 0) */
	MPP48_GPIO,		/* CPLD extension (addr 1) */
	MPP49_GPIO,		/* CPLD extension (addr 2) */
	0
};

static unsigned int net5big_v2_mpp_config[] __initdata = {
	MPP0_SPI_SCn,
	MPP1_SPI_MOSI,
	MPP2_SPI_SCK,
	MPP3_SPI_MISO,
	MPP6_SYSRST_OUTn,
	MPP7_GPO,		/* Request power-off */
	MPP8_TW_SDA,
	MPP9_TW_SCK,
	MPP10_UART0_TXD,
	MPP11_UART0_RXD,
	MPP13_GPIO,		/* Rear power switch (on|auto) */
	MPP14_GPIO,		/* USB fuse alarm */
	MPP15_GPIO,		/* Rear power switch (auto|off) */
	MPP16_GPIO,		/* SATA HDD1 power */
	MPP17_GPIO,		/* SATA HDD2 power */
	MPP20_GE1_0,
	MPP21_GE1_1,
	MPP22_GE1_2,
	MPP23_GE1_3,
	MPP24_GE1_4,
	MPP25_GE1_5,
	MPP26_GE1_6,
	MPP27_GE1_7,
	MPP28_GPIO,		/* USB enable host vbus */
	MPP29_GPIO,		/* CPLD extension ALE */
	MPP30_GE1_10,
	MPP31_GE1_11,
	MPP32_GE1_12,
	MPP33_GE1_13,
	MPP34_GPIO,		/* Rear Push button */
	MPP35_GPIO,		/* Inhibit switch power-off */
	MPP36_GPIO,		/* SATA HDD1 presence */
	MPP37_GPIO,		/* SATA HDD2 presence */
	MPP38_GPIO,		/* SATA HDD3 presence */
	MPP39_GPIO,		/* SATA HDD4 presence */
	MPP40_GPIO,		/* SATA HDD5 presence */
	MPP41_GPIO,		/* SATA HDD3 power */
	MPP42_GPIO,		/* SATA HDD4 power */
	MPP43_GPIO,		/* SATA HDD5 power */
	MPP44_GPIO,		/* CPLD extension (data 0) */
	MPP45_GPIO,		/* CPLD extension (data 1) */
	MPP46_GPIO,		/* CPLD extension (data 2) */
	MPP47_GPIO,		/* CPLD extension (addr 0) */
	MPP48_GPIO,		/* CPLD extension (addr 1) */
	MPP49_GPIO,		/* CPLD extension (addr 2) */
	0
};

#define NETXBIG_V2_GPIO_POWER_OFF		7

static void netxbig_v2_power_off(void)
{
	gpio_set_value(NETXBIG_V2_GPIO_POWER_OFF, 1);
}

static void __init netxbig_v2_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	kirkwood_init();
	if (machine_is_net2big_v2())
		kirkwood_mpp_conf(net2big_v2_mpp_config);
	else
		kirkwood_mpp_conf(net5big_v2_mpp_config);

	netxbig_v2_sata_power_init();

	kirkwood_ehci_init();
	kirkwood_ge00_init(&netxbig_v2_ge00_data);
	if (machine_is_net5big_v2())
		kirkwood_ge01_init(&netxbig_v2_ge01_data);
	kirkwood_sata_init(&netxbig_v2_sata_data);
	kirkwood_uart0_init();
	spi_register_board_info(netxbig_v2_spi_slave_info,
				ARRAY_SIZE(netxbig_v2_spi_slave_info));
	kirkwood_spi_init();
	kirkwood_i2c_init();
	i2c_register_board_info(0, netxbig_v2_i2c_info,
				ARRAY_SIZE(netxbig_v2_i2c_info));

	platform_device_register(&netxbig_v2_gpio_buttons);

	if (gpio_request(NETXBIG_V2_GPIO_POWER_OFF, "power-off") == 0 &&
	    gpio_direction_output(NETXBIG_V2_GPIO_POWER_OFF, 0) == 0)
		pm_power_off = netxbig_v2_power_off;
	else
		pr_err("netxbig_v2: failed to configure power-off GPIO\n");
}

#ifdef CONFIG_MACH_NET2BIG_V2
MACHINE_START(NET2BIG_V2, "LaCie 2Big Network v2")
	.phys_io	= KIRKWOOD_REGS_PHYS_BASE,
	.io_pg_offst	= ((KIRKWOOD_REGS_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.init_machine	= netxbig_v2_init,
	.map_io		= kirkwood_map_io,
	.init_irq	= kirkwood_init_irq,
	.timer		= &netxbig_v2_timer,
MACHINE_END
#endif

#ifdef CONFIG_MACH_NET5BIG_V2
MACHINE_START(NET5BIG_V2, "LaCie 5Big Network v2")
	.phys_io	= KIRKWOOD_REGS_PHYS_BASE,
	.io_pg_offst	= ((KIRKWOOD_REGS_VIRT_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.init_machine	= netxbig_v2_init,
	.map_io		= kirkwood_map_io,
	.init_irq	= kirkwood_init_irq,
	.timer		= &netxbig_v2_timer,
MACHINE_END
#endif
