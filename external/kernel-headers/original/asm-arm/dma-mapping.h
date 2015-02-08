#ifndef ASMARM_DMA_MAPPING_H
#define ASMARM_DMA_MAPPING_H

#ifdef __KERNEL__

#include <linux/mm.h> /* need struct page */
#include <linux/device.h>

#include <asm/scatterlist.h>

extern void consistent_sync(void *kaddr, size_t size, int rw);

static inline int dma_supported(struct device *dev, u64 mask)
{
	return dev->dma_mask && *dev->dma_mask != 0;
}

static inline int dma_set_mask(struct device *dev, u64 dma_mask)
{
	if (!dev->dma_mask || !dma_supported(dev, dma_mask))
		return -EIO;

	*dev->dma_mask = dma_mask;

	return 0;
}

static inline int dma_get_cache_alignment(void)
{
	return 32;
}

static inline int dma_is_consistent(dma_addr_t handle)
{
	return !!arch_is_coherent();
}

static inline int dma_mapping_error(dma_addr_t dma_addr)
{
	return dma_addr == ~0;
}

extern void *
dma_alloc_coherent(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp);

extern void
dma_free_coherent(struct device *dev, size_t size, void *cpu_addr,
		  dma_addr_t handle);

int dma_mmap_coherent(struct device *dev, struct vm_area_struct *vma,
		      void *cpu_addr, dma_addr_t handle, size_t size);


extern void *
dma_alloc_writecombine(struct device *dev, size_t size, dma_addr_t *handle, gfp_t gfp);

#define dma_free_writecombine(dev,size,cpu_addr,handle) \
	dma_free_coherent(dev,size,cpu_addr,handle)

int dma_mmap_writecombine(struct device *dev, struct vm_area_struct *vma,
			  void *cpu_addr, dma_addr_t handle, size_t size);


#ifndef CONFIG_DMABOUNCE
static inline dma_addr_t
dma_map_single(struct device *dev, void *cpu_addr, size_t size,
	       enum dma_data_direction dir)
{
	if (!arch_is_coherent())
		consistent_sync(cpu_addr, size, dir);

	return virt_to_dma(dev, (unsigned long)cpu_addr);
}
#else
extern dma_addr_t dma_map_single(struct device *,void *, size_t, enum dma_data_direction);
#endif

static inline dma_addr_t
dma_map_page(struct device *dev, struct page *page,
	     unsigned long offset, size_t size,
	     enum dma_data_direction dir)
{
	return dma_map_single(dev, page_address(page) + offset, size, (int)dir);
}

#ifndef CONFIG_DMABOUNCE
static inline void
dma_unmap_single(struct device *dev, dma_addr_t handle, size_t size,
		 enum dma_data_direction dir)
{
	/* nothing to do */
}
#else
extern void dma_unmap_single(struct device *, dma_addr_t, size_t, enum dma_data_direction);
#endif

static inline void
dma_unmap_page(struct device *dev, dma_addr_t handle, size_t size,
	       enum dma_data_direction dir)
{
	dma_unmap_single(dev, handle, size, (int)dir);
}

#ifndef CONFIG_DMABOUNCE
static inline int
dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
	   enum dma_data_direction dir)
{
	int i;

	for (i = 0; i < nents; i++, sg++) {
		char *virt;

		sg->dma_address = page_to_dma(dev, sg->page) + sg->offset;
		virt = page_address(sg->page) + sg->offset;

		if (!arch_is_coherent())
			consistent_sync(virt, sg->length, dir);
	}

	return nents;
}
#else
extern int dma_map_sg(struct device *, struct scatterlist *, int, enum dma_data_direction);
#endif

#ifndef CONFIG_DMABOUNCE
static inline void
dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nents,
	     enum dma_data_direction dir)
{

	/* nothing to do */
}
#else
extern void dma_unmap_sg(struct device *, struct scatterlist *, int, enum dma_data_direction);
#endif


#ifndef CONFIG_DMABOUNCE
static inline void
dma_sync_single_for_cpu(struct device *dev, dma_addr_t handle, size_t size,
			enum dma_data_direction dir)
{
	if (!arch_is_coherent())
		consistent_sync((void *)dma_to_virt(dev, handle), size, dir);
}

static inline void
dma_sync_single_for_device(struct device *dev, dma_addr_t handle, size_t size,
			   enum dma_data_direction dir)
{
	if (!arch_is_coherent())
		consistent_sync((void *)dma_to_virt(dev, handle), size, dir);
}
#else
extern void dma_sync_single_for_cpu(struct device*, dma_addr_t, size_t, enum dma_data_direction);
extern void dma_sync_single_for_device(struct device*, dma_addr_t, size_t, enum dma_data_direction);
#endif


#ifndef CONFIG_DMABOUNCE
static inline void
dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg, int nents,
		    enum dma_data_direction dir)
{
	int i;

	for (i = 0; i < nents; i++, sg++) {
		char *virt = page_address(sg->page) + sg->offset;
		if (!arch_is_coherent())
			consistent_sync(virt, sg->length, dir);
	}
}

static inline void
dma_sync_sg_for_device(struct device *dev, struct scatterlist *sg, int nents,
		       enum dma_data_direction dir)
{
	int i;

	for (i = 0; i < nents; i++, sg++) {
		char *virt = page_address(sg->page) + sg->offset;
		if (!arch_is_coherent())
			consistent_sync(virt, sg->length, dir);
	}
}
#else
extern void dma_sync_sg_for_cpu(struct device*, struct scatterlist*, int, enum dma_data_direction);
extern void dma_sync_sg_for_device(struct device*, struct scatterlist*, int, enum dma_data_direction);
#endif

#ifdef CONFIG_DMABOUNCE

extern int dmabounce_register_dev(struct device *, unsigned long, unsigned long);

extern void dmabounce_unregister_dev(struct device *);

extern int dma_needs_bounce(struct device*, dma_addr_t, size_t);
#endif /* CONFIG_DMABOUNCE */

#endif /* __KERNEL__ */
#endif
