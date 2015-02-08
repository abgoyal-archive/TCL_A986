

#ifndef __ARCH_ARM_DAVINCI_SPI_H
#define __ARCH_ARM_DAVINCI_SPI_H

enum {
	SPI_VERSION_1, /* For DM355/DM365/DM6467 */
	SPI_VERSION_2, /* For DA8xx */
};

struct davinci_spi_platform_data {
	u8	version;
	u8	num_chipselect;
	u8	wdelay;
	u8	odd_parity;
	u8	parity_enable;
	u8	wait_enable;
	u8	timer_disable;
	u8	clk_internal;
	u8	cs_hold;
	u8	intr_level;
	u8	poll_mode;
	u8	use_dma;
	u8	c2tdelay;
	u8	t2cdelay;
};

#endif	/* __ARCH_ARM_DAVINCI_SPI_H */
