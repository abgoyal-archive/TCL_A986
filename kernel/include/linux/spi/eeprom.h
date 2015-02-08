
#ifndef __LINUX_SPI_EEPROM_H
#define __LINUX_SPI_EEPROM_H

#include <linux/memory.h>

struct spi_eeprom {
	u32		byte_len;
	char		name[10];
	u16		page_size;		/* for writes */
	u16		flags;
#define	EE_ADDR1	0x0001			/*  8 bit addrs */
#define	EE_ADDR2	0x0002			/* 16 bit addrs */
#define	EE_ADDR3	0x0004			/* 24 bit addrs */
#define	EE_READONLY	0x0008			/* disallow writes */

	/* for exporting this chip's data to other kernel code */
	void (*setup)(struct memory_accessor *mem, void *context);
	void *context;
};

#endif /* __LINUX_SPI_EEPROM_H */
