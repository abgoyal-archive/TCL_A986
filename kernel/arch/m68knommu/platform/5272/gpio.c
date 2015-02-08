

#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfgpio.h>

static struct mcf_gpio_chip mcf_gpio_chips[] = {
	{
		.gpio_chip			= {
			.label			= "PA",
			.request		= mcf_gpio_request,
			.free			= mcf_gpio_free,
			.direction_input	= mcf_gpio_direction_input,
			.direction_output	= mcf_gpio_direction_output,
			.get			= mcf_gpio_get_value,
			.set			= mcf_gpio_set_value,
			.ngpio			= 16,
		},
		.pddr				= MCFSIM_PADDR,
		.podr				= MCFSIM_PADAT,
		.ppdr				= MCFSIM_PADAT,
	},
	{
		.gpio_chip			= {
			.label			= "PB",
			.request		= mcf_gpio_request,
			.free			= mcf_gpio_free,
			.direction_input	= mcf_gpio_direction_input,
			.direction_output	= mcf_gpio_direction_output,
			.get			= mcf_gpio_get_value,
			.set			= mcf_gpio_set_value,
			.base			= 16,
			.ngpio			= 16,
		},
		.pddr				= MCFSIM_PBDDR,
		.podr				= MCFSIM_PBDAT,
		.ppdr				= MCFSIM_PBDAT,
	},
	{
		.gpio_chip			= {
			.label			= "PC",
			.request		= mcf_gpio_request,
			.free			= mcf_gpio_free,
			.direction_input	= mcf_gpio_direction_input,
			.direction_output	= mcf_gpio_direction_output,
			.get			= mcf_gpio_get_value,
			.set			= mcf_gpio_set_value,
			.base			= 32,
			.ngpio			= 16,
		},
		.pddr				= MCFSIM_PCDDR,
		.podr				= MCFSIM_PCDAT,
		.ppdr				= MCFSIM_PCDAT,
	},
};

static int __init mcf_gpio_init(void)
{
	unsigned i = 0;
	while (i < ARRAY_SIZE(mcf_gpio_chips))
		(void)gpiochip_add((struct gpio_chip *)&mcf_gpio_chips[i++]);
	return 0;
}

core_initcall(mcf_gpio_init);
