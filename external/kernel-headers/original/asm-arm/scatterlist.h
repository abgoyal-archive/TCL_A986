#ifndef _ASMARM_SCATTERLIST_H
#define _ASMARM_SCATTERLIST_H

#include <asm/memory.h>
#include <asm/types.h>

struct scatterlist {
	struct page	*page;		/* buffer page			 */
	unsigned int	offset;		/* buffer offset		 */
	dma_addr_t	dma_address;	/* dma address			 */
	unsigned int	length;		/* length			 */
};

#define sg_dma_address(sg)      ((sg)->dma_address)
#define sg_dma_len(sg)          ((sg)->length)

#endif /* _ASMARM_SCATTERLIST_H */
