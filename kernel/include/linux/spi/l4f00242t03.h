

#ifndef _INCLUDE_LINUX_SPI_L4F00242T03_H_
#define _INCLUDE_LINUX_SPI_L4F00242T03_H_

struct l4f00242t03_pdata {
	unsigned int	reset_gpio;
	unsigned int	data_enable_gpio;
	const char 	*io_supply;	/* will be set to 1.8 V */
	const char 	*core_supply;	/* will be set to 2.8 V */
};

#endif /* _INCLUDE_LINUX_SPI_L4F00242T03_H_ */
