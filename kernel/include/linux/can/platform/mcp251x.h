
#ifndef __CAN_PLATFORM_MCP251X_H__
#define __CAN_PLATFORM_MCP251X_H__


#include <linux/spi/spi.h>


struct mcp251x_platform_data {
	unsigned long oscillator_frequency;
	int model;
#define CAN_MCP251X_MCP2510 0x2510
#define CAN_MCP251X_MCP2515 0x2515
	int (*board_specific_setup)(struct spi_device *spi);
	int (*transceiver_enable)(int enable);
	int (*power_enable) (int enable);
};

#endif /* __CAN_PLATFORM_MCP251X_H__ */
