

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/regs-wan.h>
#include <mach/regs-lan.h>
#include <mach/regs-hpna.h>
#include <mach/regs-switch.h>
#include <mach/regs-misc.h>



static u64 eth_dmamask = 0xffffffffUL;

static struct resource ks8695_wan_resources[] = {
	[0] = {
		.start	= KS8695_WAN_PA,
		.end	= KS8695_WAN_PA + 0x00ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "WAN RX",
		.start	= KS8695_IRQ_WAN_RX_STATUS,
		.end	= KS8695_IRQ_WAN_RX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "WAN TX",
		.start	= KS8695_IRQ_WAN_TX_STATUS,
		.end	= KS8695_IRQ_WAN_TX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.name	= "WAN Link",
		.start	= KS8695_IRQ_WAN_LINK,
		.end	= KS8695_IRQ_WAN_LINK,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.name	= "WAN PHY",
		.start	= KS8695_MISC_PA,
		.end	= KS8695_MISC_PA + 0x1f,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ks8695_wan_device = {
	.name		= "ks8695_ether",
	.id		= 0,
	.dev		= {
				.dma_mask		= &eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= ks8695_wan_resources,
	.num_resources	= ARRAY_SIZE(ks8695_wan_resources),
};


static struct resource ks8695_lan_resources[] = {
	[0] = {
		.start	= KS8695_LAN_PA,
		.end	= KS8695_LAN_PA + 0x00ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "LAN RX",
		.start	= KS8695_IRQ_LAN_RX_STATUS,
		.end	= KS8695_IRQ_LAN_RX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "LAN TX",
		.start	= KS8695_IRQ_LAN_TX_STATUS,
		.end	= KS8695_IRQ_LAN_TX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.name	= "LAN SWITCH",
		.start	= KS8695_SWITCH_PA,
		.end	= KS8695_SWITCH_PA + 0x4f,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ks8695_lan_device = {
	.name		= "ks8695_ether",
	.id		= 1,
	.dev		= {
				.dma_mask		= &eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= ks8695_lan_resources,
	.num_resources	= ARRAY_SIZE(ks8695_lan_resources),
};


static struct resource ks8695_hpna_resources[] = {
	[0] = {
		.start	= KS8695_HPNA_PA,
		.end	= KS8695_HPNA_PA + 0x00ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "HPNA RX",
		.start	= KS8695_IRQ_HPNA_RX_STATUS,
		.end	= KS8695_IRQ_HPNA_RX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "HPNA TX",
		.start	= KS8695_IRQ_HPNA_TX_STATUS,
		.end	= KS8695_IRQ_HPNA_TX_STATUS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ks8695_hpna_device = {
	.name		= "ks8695_ether",
	.id		= 2,
	.dev		= {
				.dma_mask		= &eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= ks8695_hpna_resources,
	.num_resources	= ARRAY_SIZE(ks8695_hpna_resources),
};

void __init ks8695_add_device_wan(void)
{
	platform_device_register(&ks8695_wan_device);
}

void __init ks8695_add_device_lan(void)
{
	platform_device_register(&ks8695_lan_device);
}

void __init ks8696_add_device_hpna(void)
{
	platform_device_register(&ks8695_hpna_device);
}



static struct platform_device ks8695_wdt_device = {
	.name		= "ks8695_wdt",
	.id		= -1,
	.num_resources	= 0,
};

static void __init ks8695_add_device_watchdog(void)
{
	platform_device_register(&ks8695_wdt_device);
}



#if defined(CONFIG_LEDS)
short ks8695_leds_cpu = -1;
short ks8695_leds_timer = -1;

void __init ks8695_init_leds(u8 cpu_led, u8 timer_led)
{
	/* Enable GPIO to access the LEDs */
	gpio_direction_output(cpu_led, 1);
	gpio_direction_output(timer_led, 1);

	ks8695_leds_cpu	  = cpu_led;
	ks8695_leds_timer = timer_led;
}
#else
void __init ks8695_init_leds(u8 cpu_led, u8 timer_led) {}
#endif

/* -------------------------------------------------------------------- */

static int __init ks8695_add_standard_devices(void)
{
	ks8695_add_device_watchdog();
	return 0;
}

arch_initcall(ks8695_add_standard_devices);
