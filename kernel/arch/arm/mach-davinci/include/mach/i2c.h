

#ifndef __ASM_ARCH_I2C_H
#define __ASM_ARCH_I2C_H

/* All frequencies are expressed in kHz */
struct davinci_i2c_platform_data {
	unsigned int	bus_freq;	/* standard bus frequency (kHz) */
	unsigned int	bus_delay;	/* post-transaction delay (usec) */
	unsigned int    sda_pin;        /* GPIO pin ID to use for SDA */
	unsigned int    scl_pin;        /* GPIO pin ID to use for SCL */
};

/* for board setup code */
void davinci_init_i2c(struct davinci_i2c_platform_data *);

#endif /* __ASM_ARCH_I2C_H */
