
#ifndef MMC_H
#define MMC_H

#include <linux/amba/bus.h>

int __devinit mmc_init(struct amba_device *adev);

#endif
