

#ifndef mcfgpio_h
#define mcfgpio_h

#include <linux/io.h>
#include <asm-generic/gpio.h>

struct mcf_gpio_chip {
	struct gpio_chip gpio_chip;
	void __iomem *pddr;
	void __iomem *podr;
	void __iomem *ppdr;
	void __iomem *setr;
	void __iomem *clrr;
	const u8 *gpio_to_pinmux;
};

int mcf_gpio_direction_input(struct gpio_chip *, unsigned);
int mcf_gpio_get_value(struct gpio_chip *, unsigned);
int mcf_gpio_direction_output(struct gpio_chip *, unsigned, int);
void mcf_gpio_set_value(struct gpio_chip *, unsigned, int);
void mcf_gpio_set_value_fast(struct gpio_chip *, unsigned, int);
int mcf_gpio_request(struct gpio_chip *, unsigned);
void mcf_gpio_free(struct gpio_chip *, unsigned);

#endif
