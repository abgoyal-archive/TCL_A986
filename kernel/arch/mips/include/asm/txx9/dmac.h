

#ifndef __ASM_TXX9_DMAC_H
#define __ASM_TXX9_DMAC_H

#include <linux/dmaengine.h>

#define TXX9_DMA_MAX_NR_CHANNELS	4

struct txx9dmac_platform_data {
	int	memcpy_chan;
	bool	have_64bit_regs;
};

struct txx9dmac_chan_platform_data {
	struct platform_device *dmac_dev;
};

struct txx9dmac_slave {
	u64		tx_reg;
	u64		rx_reg;
	unsigned int	reg_width;
};

void txx9_dmac_init(int id, unsigned long baseaddr, int irq,
		    const struct txx9dmac_platform_data *pdata);

#endif /* __ASM_TXX9_DMAC_H */
