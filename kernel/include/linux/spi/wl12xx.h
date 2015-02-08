

#ifndef _LINUX_SPI_WL12XX_H
#define _LINUX_SPI_WL12XX_H

struct wl12xx_platform_data {
	void (*set_power)(bool enable);
	/* SDIO only: IRQ number if WLAN_IRQ line is used, 0 for SDIO IRQs */
	int irq;
	bool use_eeprom;
};

#endif
