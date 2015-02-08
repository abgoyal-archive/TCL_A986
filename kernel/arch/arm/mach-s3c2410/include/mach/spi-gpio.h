

#ifndef __ASM_ARCH_SPIGPIO_H
#define __ASM_ARCH_SPIGPIO_H __FILE__

struct s3c2410_spigpio_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;

	int			 num_chipselect;
	int			 bus_num;

	void (*chip_select)(struct s3c2410_spigpio_info *spi, int cs);
};


#endif /* __ASM_ARCH_SPIGPIO_H */
