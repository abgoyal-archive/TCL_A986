
#ifndef __SEP_DEV_H__
#define __SEP_DEV_H__


struct sep_device {
	/* pointer to pci dev */
	struct pci_dev *pdev;

	unsigned long in_use;

	/* address of the shared memory allocated during init for SEP driver
	   (coherent alloc) */
	void *shared_addr;
	/* the physical address of the shared area */
	dma_addr_t shared_bus;

	/* restricted access region (coherent alloc) */
	dma_addr_t rar_bus;
	void *rar_addr;
	/* firmware regions: cache is at rar_addr */
	unsigned long cache_size;

	/* follows the cache */
	dma_addr_t resident_bus;
	unsigned long resident_size;
	void *resident_addr;

	/* start address of the access to the SEP registers from driver */
	void __iomem *reg_addr;
	/* transaction counter that coordinates the transactions between SEP and HOST */
	unsigned long send_ct;
	/* counter for the messages from sep */
	unsigned long reply_ct;
	/* counter for the number of bytes allocated in the pool for the current
	   transaction */
	unsigned long data_pool_bytes_allocated;

	/* array of pointers to the pages that represent input data for the synchronic
	   DMA action */
	struct page **in_page_array;

	/* array of pointers to the pages that represent out data for the synchronic
	   DMA action */
	struct page **out_page_array;

	/* number of pages in the sep_in_page_array */
	unsigned long in_num_pages;

	/* number of pages in the sep_out_page_array */
	unsigned long out_num_pages;

	/* global data for every flow */
	struct sep_flow_context_t flows[SEP_DRIVER_NUM_FLOWS];

	/* pointer to the workqueue that handles the flow done interrupts */
	struct workqueue_struct *flow_wq;

};

static struct sep_device *sep_dev;

static inline void sep_write_reg(struct sep_device *dev, int reg, u32 value)
{
	void __iomem *addr = dev->reg_addr + reg;
	writel(value, addr);
}

static inline u32 sep_read_reg(struct sep_device *dev, int reg)
{
	void __iomem *addr = dev->reg_addr + reg;
	return readl(addr);
}

/* wait for SRAM write complete(indirect write */
static inline void sep_wait_sram_write(struct sep_device *dev)
{
	u32 reg_val;
	do
		reg_val = sep_read_reg(dev, HW_SRAM_DATA_READY_REG_ADDR);
	while (!(reg_val & 1));
}


#endif
