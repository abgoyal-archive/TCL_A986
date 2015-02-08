

#ifndef EFX_SPI_H
#define EFX_SPI_H

#include "net_driver.h"


#define SPI_WRSR 0x01		/* Write status register */
#define SPI_WRITE 0x02		/* Write data to memory array */
#define SPI_READ 0x03		/* Read data from memory array */
#define SPI_WRDI 0x04		/* Reset write enable latch */
#define SPI_RDSR 0x05		/* Read status register */
#define SPI_WREN 0x06		/* Set write enable latch */
#define SPI_SST_EWSR 0x50	/* SST: Enable write to status register */

#define SPI_STATUS_WPEN 0x80	/* Write-protect pin enabled */
#define SPI_STATUS_BP2 0x10	/* Block protection bit 2 */
#define SPI_STATUS_BP1 0x08	/* Block protection bit 1 */
#define SPI_STATUS_BP0 0x04	/* Block protection bit 0 */
#define SPI_STATUS_WEN 0x02	/* State of the write enable latch */
#define SPI_STATUS_NRDY 0x01	/* Device busy flag */

struct efx_spi_device {
	int device_id;
	unsigned int size;
	unsigned int addr_len;
	unsigned int munge_address:1;
	u8 erase_command;
	unsigned int erase_size;
	unsigned int block_size;
};

int falcon_spi_cmd(struct efx_nic *efx,
		   const struct efx_spi_device *spi, unsigned int command,
		   int address, const void* in, void *out, size_t len);
int falcon_spi_wait_write(struct efx_nic *efx,
			  const struct efx_spi_device *spi);
int falcon_spi_read(struct efx_nic *efx,
		    const struct efx_spi_device *spi, loff_t start,
		    size_t len, size_t *retlen, u8 *buffer);
int falcon_spi_write(struct efx_nic *efx,
		     const struct efx_spi_device *spi, loff_t start,
		     size_t len, size_t *retlen, const u8 *buffer);

#define FALCON_NVCONFIG_END 0x400U
#define FALCON_FLASH_BOOTCODE_START 0x8000U
#define EFX_EEPROM_BOOTCONFIG_START 0x800U
#define EFX_EEPROM_BOOTCONFIG_END 0x1800U

#endif /* EFX_SPI_H */
