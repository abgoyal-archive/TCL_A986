
#ifndef _VIDEOBUF_DMA_SG_H
#define _VIDEOBUF_DMA_SG_H

#include <media/videobuf-core.h>

/* --------------------------------------------------------------------- */

struct scatterlist *videobuf_vmalloc_to_sg(unsigned char *virt, int nr_pages);

struct scatterlist *videobuf_pages_to_sg(struct page **pages, int nr_pages,
					 int offset);

/* --------------------------------------------------------------------- */


struct videobuf_dmabuf {
	u32                 magic;

	/* for userland buffer */
	int                 offset;
	struct page         **pages;

	/* for kernel buffers */
	void                *vmalloc;

	/* for overlay buffers (pci-pci dma) */
	dma_addr_t          bus_addr;

	/* common */
	struct scatterlist  *sglist;
	int                 sglen;
	int                 nr_pages;
	int                 direction;
};

struct videobuf_dma_sg_memory {
	u32                 magic;

	/* for mmap'ed buffers */
	struct videobuf_dmabuf  dma;
};

void videobuf_dma_init(struct videobuf_dmabuf *dma);
int videobuf_dma_init_user(struct videobuf_dmabuf *dma, int direction,
			   unsigned long data, unsigned long size);
int videobuf_dma_init_kernel(struct videobuf_dmabuf *dma, int direction,
			     int nr_pages);
int videobuf_dma_init_overlay(struct videobuf_dmabuf *dma, int direction,
			      dma_addr_t addr, int nr_pages);
int videobuf_dma_free(struct videobuf_dmabuf *dma);

int videobuf_dma_map(struct videobuf_queue *q, struct videobuf_dmabuf *dma);
int videobuf_dma_unmap(struct videobuf_queue *q, struct videobuf_dmabuf *dma);
struct videobuf_dmabuf *videobuf_to_dma(struct videobuf_buffer *buf);

void *videobuf_sg_alloc(size_t size);

void videobuf_queue_sg_init(struct videobuf_queue *q,
			 const struct videobuf_queue_ops *ops,
			 struct device *dev,
			 spinlock_t *irqlock,
			 enum v4l2_buf_type type,
			 enum v4l2_field field,
			 unsigned int msize,
			 void *priv);

int videobuf_sg_dma_map(struct device *dev, struct videobuf_dmabuf *dma);
int videobuf_sg_dma_unmap(struct device *dev, struct videobuf_dmabuf *dma);

#endif /* _VIDEOBUF_DMA_SG_H */

