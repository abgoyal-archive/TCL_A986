

#ifndef _XILINX_SPI_H_
#define _XILINX_SPI_H_

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#define XILINX_SPI_NAME "xilinx_spi"

struct spi_master *xilinx_spi_init(struct device *dev, struct resource *mem,
	u32 irq, s16 bus_num);

void xilinx_spi_deinit(struct spi_master *master);
#endif
