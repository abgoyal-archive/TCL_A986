

#ifndef __LINUX_SPI_ORION_SPI_H
#define __LINUX_SPI_ORION_SPI_H

struct orion_spi_info {
	u32	tclk;		/* no <linux/clk.h> support yet */
	u32	enable_clock_fix;
};


#endif
