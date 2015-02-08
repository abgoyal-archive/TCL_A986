

#ifndef MMC_SDHCI_SPEAR_H
#define MMC_SDHCI_SPEAR_H

#include <linux/platform_device.h>
struct sdhci_plat_data {
	int card_power_gpio;
	int power_active_high;
	int power_always_enb;
	int card_int_gpio;
};

/* This function is used to set platform_data field of pdev->dev */
static inline void
sdhci_set_plat_data(struct platform_device *pdev, struct sdhci_plat_data *data)
{
	pdev->dev.platform_data = data;
}

#endif /* MMC_SDHCI_SPEAR_H */
