

#include <linux/delay.h>
#include <linux/mmc/host.h>
#include "sdhci-of.h"
#include "sdhci.h"


#define SDHCI_HLWD_WRITE_DELAY	5 /* usecs */

static void sdhci_hlwd_writel(struct sdhci_host *host, u32 val, int reg)
{
	sdhci_be32bs_writel(host, val, reg);
	udelay(SDHCI_HLWD_WRITE_DELAY);
}

static void sdhci_hlwd_writew(struct sdhci_host *host, u16 val, int reg)
{
	sdhci_be32bs_writew(host, val, reg);
	udelay(SDHCI_HLWD_WRITE_DELAY);
}

static void sdhci_hlwd_writeb(struct sdhci_host *host, u8 val, int reg)
{
	sdhci_be32bs_writeb(host, val, reg);
	udelay(SDHCI_HLWD_WRITE_DELAY);
}

struct sdhci_of_data sdhci_hlwd = {
	.quirks = SDHCI_QUIRK_32BIT_DMA_ADDR |
		  SDHCI_QUIRK_32BIT_DMA_SIZE,
	.ops = {
		.read_l = sdhci_be32bs_readl,
		.read_w = sdhci_be32bs_readw,
		.read_b = sdhci_be32bs_readb,
		.write_l = sdhci_hlwd_writel,
		.write_w = sdhci_hlwd_writew,
		.write_b = sdhci_hlwd_writeb,
	},
};
