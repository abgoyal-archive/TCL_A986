

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/mach-au1x00/au1000.h>
#include <asm/mach-au1x00/gpio.h>

static int gpio2_get(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio2_get_value(offset + ALCHEMY_GPIO2_BASE);
}

static void gpio2_set(struct gpio_chip *chip, unsigned offset, int value)
{
	alchemy_gpio2_set_value(offset + ALCHEMY_GPIO2_BASE, value);
}

static int gpio2_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio2_direction_input(offset + ALCHEMY_GPIO2_BASE);
}

static int gpio2_direction_output(struct gpio_chip *chip, unsigned offset,
				  int value)
{
	return alchemy_gpio2_direction_output(offset + ALCHEMY_GPIO2_BASE,
						value);
}

static int gpio2_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio2_to_irq(offset + ALCHEMY_GPIO2_BASE);
}


static int gpio1_get(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio1_get_value(offset + ALCHEMY_GPIO1_BASE);
}

static void gpio1_set(struct gpio_chip *chip,
				unsigned offset, int value)
{
	alchemy_gpio1_set_value(offset + ALCHEMY_GPIO1_BASE, value);
}

static int gpio1_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio1_direction_input(offset + ALCHEMY_GPIO1_BASE);
}

static int gpio1_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	return alchemy_gpio1_direction_output(offset + ALCHEMY_GPIO1_BASE,
					     value);
}

static int gpio1_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return alchemy_gpio1_to_irq(offset + ALCHEMY_GPIO1_BASE);
}

struct gpio_chip alchemy_gpio_chip[] = {
	[0] = {
		.label			= "alchemy-gpio1",
		.direction_input	= gpio1_direction_input,
		.direction_output	= gpio1_direction_output,
		.get			= gpio1_get,
		.set			= gpio1_set,
		.to_irq			= gpio1_to_irq,
		.base			= ALCHEMY_GPIO1_BASE,
		.ngpio			= ALCHEMY_GPIO1_NUM,
	},
	[1] = {
		.label                  = "alchemy-gpio2",
		.direction_input        = gpio2_direction_input,
		.direction_output       = gpio2_direction_output,
		.get                    = gpio2_get,
		.set                    = gpio2_set,
		.to_irq			= gpio2_to_irq,
		.base                   = ALCHEMY_GPIO2_BASE,
		.ngpio                  = ALCHEMY_GPIO2_NUM,
	},
};

static int __init alchemy_gpiolib_init(void)
{
	gpiochip_add(&alchemy_gpio_chip[0]);
	if (alchemy_get_cputype() != ALCHEMY_CPU_AU1000)
		gpiochip_add(&alchemy_gpio_chip[1]);

	return 0;
}
arch_initcall(alchemy_gpiolib_init);
