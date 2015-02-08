

#ifndef ASM_SIU_H
#define ASM_SIU_H

struct device;

struct siu_platform {
	struct device *dma_dev;
	unsigned int dma_slave_tx_a;
	unsigned int dma_slave_rx_a;
	unsigned int dma_slave_tx_b;
	unsigned int dma_slave_rx_b;
};

#endif /* ASM_SIU_H */
