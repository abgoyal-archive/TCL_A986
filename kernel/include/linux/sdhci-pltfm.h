

#ifndef _SDHCI_PLTFM_H
#define _SDHCI_PLTFM_H

struct sdhci_ops;
struct sdhci_host;

struct sdhci_pltfm_data {
	struct sdhci_ops *ops;
	unsigned int quirks;
	int (*init)(struct sdhci_host *host);
	void (*exit)(struct sdhci_host *host);
};

#endif /* _SDHCI_PLTFM_H */
