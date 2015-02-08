
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/page.h>
#include "proc-macros.S"

ENTRY(v3_flush_user_cache_all)
	/* FALLTHROUGH */
ENTRY(v3_flush_kern_cache_all)
	/* FALLTHROUGH */

ENTRY(v3_flush_user_cache_range)
	mov	ip, #0
	mcreq	p15, 0, ip, c7, c0, 0		@ flush ID cache
	mov	pc, lr

ENTRY(v3_coherent_kern_range)
	/* FALLTHROUGH */

ENTRY(v3_coherent_user_range)
	mov	pc, lr

ENTRY(v3_flush_kern_dcache_area)
	/* FALLTHROUGH */

ENTRY(v3_dma_flush_range)
	mov	r0, #0
	mcr	p15, 0, r0, c7, c0, 0		@ flush ID cache
	mov	pc, lr

ENTRY(v3_dma_unmap_area)
	teq	r2, #DMA_TO_DEVICE
	bne	v3_dma_flush_range
	/* FALLTHROUGH */

ENTRY(v3_dma_map_area)
	mov	pc, lr
ENDPROC(v3_dma_unmap_area)
ENDPROC(v3_dma_map_area)

	__INITDATA

	.type	v3_cache_fns, #object
ENTRY(v3_cache_fns)
	.long	v3_flush_kern_cache_all
	.long	v3_flush_user_cache_all
	.long	v3_flush_user_cache_range
	.long	v3_coherent_kern_range
	.long	v3_coherent_user_range
	.long	v3_flush_kern_dcache_area
	.long	v3_dma_map_area
	.long	v3_dma_unmap_area
	.long	v3_dma_flush_range
	.size	v3_cache_fns, . - v3_cache_fns
