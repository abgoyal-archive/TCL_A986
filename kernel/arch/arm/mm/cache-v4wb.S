
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/memory.h>
#include <asm/page.h>
#include "proc-macros.S"

#define CACHE_DLINESIZE	32

#if defined(CONFIG_CPU_SA110)
# define CACHE_DSIZE	16384
#elif defined(CONFIG_CPU_SA1100)
# define CACHE_DSIZE	8192
#else
# error Unknown cache size
#endif

#define CACHE_DLIMIT	(CACHE_DSIZE * 4)

	.data
flush_base:
	.long	FLUSH_BASE
	.text

ENTRY(v4wb_flush_user_cache_all)
	/* FALLTHROUGH */
ENTRY(v4wb_flush_kern_cache_all)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate I cache
__flush_whole_cache:
	ldr	r3, =flush_base
	ldr	r1, [r3, #0]
	eor	r1, r1, #CACHE_DSIZE
	str	r1, [r3, #0]
	add	r2, r1, #CACHE_DSIZE
1:	ldr	r3, [r1], #32
	cmp	r1, r2
	blo	1b
#ifdef FLUSH_BASE_MINICACHE
	add	r2, r2, #FLUSH_BASE_MINICACHE - FLUSH_BASE
	sub	r1, r2, #512			@ only 512 bytes
1:	ldr	r3, [r1], #32
	cmp	r1, r2
	blo	1b
#endif
	mcr	p15, 0, ip, c7, c10, 4		@ drain write buffer
	mov	pc, lr

ENTRY(v4wb_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	tst	r2, #VM_EXEC			@ executable region?
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache

	cmp	r3, #CACHE_DLIMIT		@ total size >= limit?
	bhs	__flush_whole_cache		@ flush whole D cache

1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c10, 4		@ drain write buffer
	mov	pc, lr

ENTRY(v4wb_flush_kern_dcache_area)
	add	r1, r0, r1
	/* fall through */

ENTRY(v4wb_coherent_kern_range)
	/* fall through */

ENTRY(v4wb_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	ip, #0
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mov	pc, lr


v4wb_dma_inv_range:
	tst	r0, #CACHE_DLINESIZE - 1
	bic	r0, r0, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
	tst	r1, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean D entry
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

v4wb_dma_clean_range:
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

	.globl	v4wb_dma_flush_range
	.set	v4wb_dma_flush_range, v4wb_coherent_kern_range

ENTRY(v4wb_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	v4wb_dma_clean_range
	bcs	v4wb_dma_inv_range
	b	v4wb_dma_flush_range
ENDPROC(v4wb_dma_map_area)

ENTRY(v4wb_dma_unmap_area)
	mov	pc, lr
ENDPROC(v4wb_dma_unmap_area)

	__INITDATA

	.type	v4wb_cache_fns, #object
ENTRY(v4wb_cache_fns)
	.long	v4wb_flush_kern_cache_all
	.long	v4wb_flush_user_cache_all
	.long	v4wb_flush_user_cache_range
	.long	v4wb_coherent_kern_range
	.long	v4wb_coherent_user_range
	.long	v4wb_flush_kern_dcache_area
	.long	v4wb_dma_map_area
	.long	v4wb_dma_unmap_area
	.long	v4wb_dma_flush_range
	.size	v4wb_cache_fns, . - v4wb_cache_fns
