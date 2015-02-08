

#ifndef __SDHCI_OF_H
#define __SDHCI_OF_H

#include <linux/types.h>
#include "sdhci.h"

struct sdhci_of_data {
	unsigned int quirks;
	struct sdhci_ops ops;
};

struct sdhci_of_host {
	unsigned int clock;
	u16 xfer_mode_shadow;
};

extern u32 sdhci_be32bs_readl(struct sdhci_host *host, int reg);
extern u16 sdhci_be32bs_readw(struct sdhci_host *host, int reg);
extern u8 sdhci_be32bs_readb(struct sdhci_host *host, int reg);
extern void sdhci_be32bs_writel(struct sdhci_host *host, u32 val, int reg);
extern void sdhci_be32bs_writew(struct sdhci_host *host, u16 val, int reg);
extern void sdhci_be32bs_writeb(struct sdhci_host *host, u8 val, int reg);

extern struct sdhci_of_data sdhci_esdhc;
extern struct sdhci_of_data sdhci_hlwd;

#endif /* __SDHCI_OF_H */
