

#include <linux/serial_8250.h>
#include <linux/dm9000.h>
#include <linux/gpio.h>

#include <plat/i2c.h>
#include <plat/pxa3xx_nand.h>

#include <mach/pxafb.h>
#include <mach/mmc.h>
#include <mach/ohci.h>
#include <mach/pxa320.h>

#include <mach/mxm8x10.h>

#include "devices.h"
#include "generic.h"


static mfp_cfg_t mfp_cfg[] __initdata = {
	/* USB */
	GPIO10_UTM_CLK,
	GPIO49_U2D_PHYDATA_0,
	GPIO50_U2D_PHYDATA_1,
	GPIO51_U2D_PHYDATA_2,
	GPIO52_U2D_PHYDATA_3,
	GPIO53_U2D_PHYDATA_4,
	GPIO54_U2D_PHYDATA_5,
	GPIO55_U2D_PHYDATA_6,
	GPIO56_U2D_PHYDATA_7,
	GPIO58_UTM_RXVALID,
	GPIO59_UTM_RXACTIVE,
	GPIO60_U2D_RXERROR,
	GPIO61_U2D_OPMODE0,
	GPIO62_U2D_OPMODE1,
	GPIO71_GPIO, /* USBD_INT */
	GPIO73_UTM_TXREADY,
	GPIO83_U2D_TXVALID,
	GPIO98_U2D_RESET,
	GPIO99_U2D_XCVR_SEL,
	GPIO100_U2D_TERM_SEL,
	GPIO101_U2D_SUSPEND,
	GPIO102_UTM_LINESTATE_0,
	GPIO103_UTM_LINESTATE_1,
	GPIO4_2_GPIO | MFP_PULL_HIGH, /* UTM_PULLUP */

	/* DM9000 */
	GPIO1_GPIO,
	GPIO9_GPIO,
	GPIO36_GPIO,

	/* AC97 */
	GPIO35_AC97_SDATA_IN_0,
	GPIO37_AC97_SDATA_OUT,
	GPIO38_AC97_SYNC,
	GPIO39_AC97_BITCLK,
	GPIO40_AC97_nACRESET,

	/* UARTS */
	GPIO41_UART1_RXD,
	GPIO42_UART1_TXD,
	GPIO43_UART1_CTS,
	GPIO44_UART1_DCD,
	GPIO45_UART1_DSR,
	GPIO46_UART1_RI,
	GPIO47_UART1_DTR,
	GPIO48_UART1_RTS,

	GPIO109_UART2_RTS,
	GPIO110_UART2_RXD,
	GPIO111_UART2_TXD,
	GPIO112_UART2_CTS,

	GPIO105_UART3_CTS,
	GPIO106_UART3_RTS,
	GPIO107_UART3_TXD,
	GPIO108_UART3_RXD,

	GPIO78_GPIO,
	GPIO79_GPIO,
	GPIO80_GPIO,
	GPIO81_GPIO,

	/* I2C */
	GPIO32_I2C_SCL,
	GPIO33_I2C_SDA,

	/* MMC */
	GPIO18_MMC1_DAT0,
	GPIO19_MMC1_DAT1,
	GPIO20_MMC1_DAT2,
	GPIO21_MMC1_DAT3,
	GPIO22_MMC1_CLK,
	GPIO23_MMC1_CMD,
	GPIO72_GPIO | MFP_PULL_HIGH, /* Card Detect */
	GPIO84_GPIO | MFP_PULL_LOW, /* Write Protect */

	/* IRQ */
	GPIO74_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ1 */
	GPIO75_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ2 */
	GPIO76_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ3 */
	GPIO77_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ4 */
	GPIO78_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ5 */
	GPIO79_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ6 */
	GPIO80_GPIO | MFP_LPM_EDGE_RISE, /* EXT_IRQ7 */
	GPIO81_GPIO | MFP_LPM_EDGE_RISE  /* EXT_IRQ8 */
};

/* MMC/MCI Support */
#if defined(CONFIG_MMC)
static struct pxamci_platform_data mxm_8x10_mci_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.detect_delay_ms = 10,
	.gpio_card_detect = MXM_8X10_SD_nCD,
	.gpio_card_ro = MXM_8X10_SD_WP,
	.gpio_power = -1
};

void __init mxm_8x10_mmc_init(void)
{
	pxa_set_mci_info(&mxm_8x10_mci_platform_data);
}
#endif

/* USB Open Host Controler Interface */
static struct pxaohci_platform_data mxm_8x10_ohci_platform_data = {
	.port_mode = PMM_NPS_MODE,
	.flags = ENABLE_PORT_ALL
};

void __init mxm_8x10_usb_host_init(void)
{
	pxa_set_ohci_info(&mxm_8x10_ohci_platform_data);
}

/* AC97 Sound Support */
static struct platform_device mxm_8x10_ac97_device = {
	.name = "pxa2xx-ac97"
};

void __init mxm_8x10_ac97_init(void)
{
	platform_device_register(&mxm_8x10_ac97_device);
}

/* NAND flash Support */
#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
#define NAND_BLOCK_SIZE SZ_128K
#define NB(x)           (NAND_BLOCK_SIZE * (x))
static struct mtd_partition mxm_8x10_nand_partitions[] = {
	[0] = {
	       .name = "boot",
	       .size = NB(0x002),
	       .offset = NB(0x000),
	       .mask_flags = MTD_WRITEABLE
	},
	[1] = {
	       .name = "kernel",
	       .size = NB(0x010),
	       .offset = NB(0x002),
	       .mask_flags = MTD_WRITEABLE
	},
	[2] = {
	       .name = "root",
	       .size = NB(0x36c),
	       .offset = NB(0x012)
	},
	[3] = {
	       .name = "bbt",
	       .size = NB(0x082),
	       .offset = NB(0x37e),
	       .mask_flags = MTD_WRITEABLE
	}
};

static struct pxa3xx_nand_platform_data mxm_8x10_nand_info = {
	.enable_arbiter = 1,
	.keep_config = 1,
	.parts = mxm_8x10_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxm_8x10_nand_partitions)
};

static void __init mxm_8x10_nand_init(void)
{
	pxa3xx_set_nand_info(&mxm_8x10_nand_info);
}
#else
static inline void mxm_8x10_nand_init(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

/* Ethernet support: Davicom DM9000 */
static struct resource dm9k_resources[] = {
	[0] = {
	       .start = MXM_8X10_ETH_PHYS + 0x300,
	       .end = MXM_8X10_ETH_PHYS + 0x300,
	       .flags = IORESOURCE_MEM
	},
	[1] = {
	       .start = MXM_8X10_ETH_PHYS + 0x308,
	       .end = MXM_8X10_ETH_PHYS + 0x308,
	       .flags = IORESOURCE_MEM
	},
	[2] = {
	       .start = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO9)),
	       .end = gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO9)),
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE
	}
};

static struct dm9000_plat_data dm9k_plat_data = {
	.flags = DM9000_PLATF_16BITONLY
};

static struct platform_device dm9k_device = {
	.name = "dm9000",
	.id = 0,
	.num_resources = ARRAY_SIZE(dm9k_resources),
	.resource = dm9k_resources,
	.dev = {
		.platform_data = &dm9k_plat_data
	}
};

static void __init mxm_8x10_ethernet_init(void)
{
	platform_device_register(&dm9k_device);
}

/* PXA UARTs */
static void __init mxm_8x10_uarts_init(void)
{
	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);
}

/* I2C and Real Time Clock */
static struct i2c_board_info __initdata mxm_8x10_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ds1337", 0x68)
	}
};

static void __init mxm_8x10_i2c_init(void)
{
	i2c_register_board_info(0, mxm_8x10_i2c_devices,
				ARRAY_SIZE(mxm_8x10_i2c_devices));
	pxa_set_i2c_info(NULL);
}

void __init mxm_8x10_barebones_init(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(mfp_cfg));

	mxm_8x10_uarts_init();
	mxm_8x10_nand_init();
	mxm_8x10_i2c_init();
	mxm_8x10_ethernet_init();
}
