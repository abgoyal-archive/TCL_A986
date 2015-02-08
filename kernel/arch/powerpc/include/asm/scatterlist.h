
#ifndef _ASM_POWERPC_SCATTERLIST_H
#define _ASM_POWERPC_SCATTERLIST_H

#include <asm/dma.h>
#include <asm-generic/scatterlist.h>

#ifdef __powerpc64__
#define ISA_DMA_THRESHOLD	(~0UL)
#endif
#define ARCH_HAS_SG_CHAIN

#endif /* _ASM_POWERPC_SCATTERLIST_H */
