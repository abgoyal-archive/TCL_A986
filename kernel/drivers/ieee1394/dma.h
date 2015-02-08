

#ifndef IEEE1394_DMA_H
#define IEEE1394_DMA_H

#include <asm/types.h>

struct file;
struct pci_dev;
struct scatterlist;
struct vm_area_struct;

struct dma_prog_region {
	unsigned char *kvirt;
	struct pci_dev *dev;
	unsigned int n_pages;
	dma_addr_t bus_addr;
};

/* clear out all fields but do not allocate any memory */
void dma_prog_region_init(struct dma_prog_region *prog);
int dma_prog_region_alloc(struct dma_prog_region *prog, unsigned long n_bytes,
			  struct pci_dev *dev);
void dma_prog_region_free(struct dma_prog_region *prog);

static inline dma_addr_t dma_prog_region_offset_to_bus(
		struct dma_prog_region *prog, unsigned long offset)
{
	return prog->bus_addr + offset;
}

struct dma_region {
	unsigned char *kvirt;
	struct pci_dev *dev;
	unsigned int n_pages;
	unsigned int n_dma_pages;
	struct scatterlist *sglist;
	int direction;
};

void dma_region_init(struct dma_region *dma);
int dma_region_alloc(struct dma_region *dma, unsigned long n_bytes,
		     struct pci_dev *dev, int direction);
void dma_region_free(struct dma_region *dma);
void dma_region_sync_for_cpu(struct dma_region *dma, unsigned long offset,
			     unsigned long len);
void dma_region_sync_for_device(struct dma_region *dma, unsigned long offset,
				unsigned long len);
int  dma_region_mmap(struct dma_region *dma, struct file *file,
		     struct vm_area_struct *vma);
dma_addr_t dma_region_offset_to_bus(struct dma_region *dma,
				    unsigned long offset);

#define dma_region_i(_dma, _type, _index) \
	( ((_type*) ((_dma)->kvirt)) + (_index) )

#endif /* IEEE1394_DMA_H */
