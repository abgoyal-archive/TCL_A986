

#ifndef PXA2XX_SPI_H_
#define PXA2XX_SPI_H_

#define PXA2XX_CS_ASSERT (0x01)
#define PXA2XX_CS_DEASSERT (0x02)

/* device.platform_data for SSP controller devices */
struct pxa2xx_spi_master {
	u32 clock_enable;
	u16 num_chipselect;
	u8 enable_dma;
};

struct pxa2xx_spi_chip {
	u8 tx_threshold;
	u8 rx_threshold;
	u8 dma_burst_size;
	u32 timeout;
	u8 enable_loopback;
	int gpio_cs;
	void (*cs_control)(u32 command);
};

extern void pxa2xx_set_spi_info(unsigned id, struct pxa2xx_spi_master *info);

#endif /*PXA2XX_SPI_H_*/
