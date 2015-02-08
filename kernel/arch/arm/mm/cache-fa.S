
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/memory.h>
#include <asm/page.h>

#include "proc-macros.S"

#define CACHE_DLINESIZE	16

#ifdef CONFIG_ARCH_GEMINI
#define CACHE_DSIZE	8192
#else
#define CACHE_DSIZE	16384 
#endif 

/* FIXME: put optimal value here. Current one is just estimation */
#define CACHE_DLIMIT	(CACHE_DSIZE * 2)

ENTRY(fa_flush_user_cache_all)
	/* FALLTHROUGH */
ENTRY(fa_flush_kern_cache_all)
	mov	ip, #0
	mov	r2, #VM_EXEC
__flush_whole_cache:
	mcr	p15, 0, ip, c7, c14, 0		@ clean/invalidate D cache
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcrne	p15, 0, ip, c7, c5, 6		@ invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ drain write buffer
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
	mov	pc, lr

ENTRY(fa_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT		@ total size >= limit?
	bhs	__flush_whole_cache		@ flush whole D cache

1:	tst	r2, #VM_EXEC
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I line
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 6		@ invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ data write barrier
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
	mov	pc, lr

ENTRY(fa_coherent_kern_range)
	/* fall through */

ENTRY(fa_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 6		@ invalidate BTB
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
	mov	pc, lr

ENTRY(fa_flush_kern_dcache_area)
	add	r1, r0, r1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean & invalidate D line
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

fa_dma_inv_range:
	tst	r0, #CACHE_DLINESIZE - 1
	bic	r0, r0, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r0, c7, c14, 1		@ clean & invalidate D entry
	tst	r1, #CACHE_DLINESIZE - 1
	bic	r1, r1, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r1, c7, c14, 1		@ clean & invalidate D entry
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

fa_dma_clean_range:
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0	
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

ENTRY(fa_dma_flush_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean & invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0	
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

ENTRY(fa_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	fa_dma_clean_range
	bcs	fa_dma_inv_range
	b	fa_dma_flush_range
ENDPROC(fa_dma_map_area)

ENTRY(fa_dma_unmap_area)
	mov	pc, lr
ENDPROC(fa_dma_unmap_area)

	__INITDATA

	.type	fa_cache_fns, #object
ENTRY(fa_cache_fns)
	.long	fa_flush_kern_cache_all
	.long	fa_flush_user_cache_all
	.long	fa_flush_user_cache_range
	.long	fa_coherent_kern_range
	.long	fa_coherent_user_range
	.long	fa_flush_kern_dcache_area
	.long	fa_dma_map_area
	.long	fa_dma_unmap_area
	.long	fa_dma_flush_range
	.size	fa_cache_fns, . - fa_cache_fns
