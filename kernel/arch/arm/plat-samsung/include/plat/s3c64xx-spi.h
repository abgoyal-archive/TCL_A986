

#ifndef __S3C64XX_PLAT_SPI_H
#define __S3C64XX_PLAT_SPI_H

struct s3c64xx_spi_csinfo {
	u8 fb_delay;
	unsigned line;
	void (*set_level)(unsigned line_id, int lvl);
};

struct s3c64xx_spi_info {
	int src_clk_nr;
	char *src_clk_name;

	int num_cs;

	int (*cfg_gpio)(struct platform_device *pdev);

	/* Following two fields are for future compatibility */
	int fifo_lvl_mask;
	int rx_lvl_offset;
	int high_speed;
};

extern void s3c64xx_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);
extern void s5pc100_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);
extern void s5pv210_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);
extern void s5p6440_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);
extern void s5p6442_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);

#endif /* __S3C64XX_PLAT_SPI_H */
