

#ifndef _ASM_MICROBLAZE_DEVICE_H
#define _ASM_MICROBLAZE_DEVICE_H

struct device_node;

struct dev_archdata {
	/* DMA operations on that device */
	struct dma_map_ops	*dma_ops;
	void                    *dma_data;
};

struct pdev_archdata {
	u64 dma_mask;
};

#endif /* _ASM_MICROBLAZE_DEVICE_H */


