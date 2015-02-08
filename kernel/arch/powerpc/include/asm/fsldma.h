

#ifndef __ARCH_POWERPC_ASM_FSLDMA_H__
#define __ARCH_POWERPC_ASM_FSLDMA_H__

#include <linux/dmaengine.h>


struct fsl_dma_hw_addr {
	struct list_head entry;

	dma_addr_t address;
	size_t length;
};

struct fsl_dma_slave {

	/* List of hardware address/length pairs */
	struct list_head addresses;

	/* Support for extra controller features */
	unsigned int request_count;
	unsigned int src_loop_size;
	unsigned int dst_loop_size;
	bool external_start;
	bool external_pause;
};

static inline int fsl_dma_slave_append(struct fsl_dma_slave *slave,
				       dma_addr_t address, size_t length)
{
	struct fsl_dma_hw_addr *addr;

	addr = kzalloc(sizeof(*addr), GFP_ATOMIC);
	if (!addr)
		return -ENOMEM;

	INIT_LIST_HEAD(&addr->entry);
	addr->address = address;
	addr->length = length;

	list_add_tail(&addr->entry, &slave->addresses);
	return 0;
}

static inline void fsl_dma_slave_free(struct fsl_dma_slave *slave)
{
	struct fsl_dma_hw_addr *addr, *tmp;

	if (slave) {
		list_for_each_entry_safe(addr, tmp, &slave->addresses, entry) {
			list_del(&addr->entry);
			kfree(addr);
		}

		kfree(slave);
	}
}

static inline struct fsl_dma_slave *fsl_dma_slave_alloc(gfp_t gfp)
{
	struct fsl_dma_slave *slave;

	slave = kzalloc(sizeof(*slave), gfp);
	if (!slave)
		return NULL;

	INIT_LIST_HEAD(&slave->addresses);
	return slave;
}

#endif /* __ARCH_POWERPC_ASM_FSLDMA_H__ */
