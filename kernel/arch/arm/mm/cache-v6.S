
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/unwind.h>

#include "proc-macros.S"

#define HARVARD_CACHE
#define CACHE_LINE_SIZE		32
#define D_CACHE_LINE_SIZE	32
#define BTB_FLUSH_SIZE		8

#ifdef CONFIG_ARM_ERRATA_411920
ENTRY(v6_icache_inval_all)
	mov	r0, #0
	mrs	r1, cpsr
	cpsid	ifa				@ disable interrupts
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate entire I-cache
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate entire I-cache
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate entire I-cache
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate entire I-cache
	msr	cpsr_cx, r1			@ restore interrupts
	.rept	11				@ ARM Ltd recommends at least
	nop					@ 11 NOPs
	.endr
	mov	pc, lr
#endif

ENTRY(v6_flush_kern_cache_all)
	mov	r0, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c14, 0		@ D cache clean+invalidate
#ifndef CONFIG_ARM_ERRATA_411920
	mcr	p15, 0, r0, c7, c5, 0		@ I+BTB cache invalidate
#else
	b	v6_icache_inval_all
#endif
#else
	mcr	p15, 0, r0, c7, c15, 0		@ Cache clean+invalidate
#endif
	mov	pc, lr

ENTRY(v6_flush_user_cache_all)
	/*FALLTHROUGH*/

ENTRY(v6_flush_user_cache_range)
	mov	pc, lr

ENTRY(v6_coherent_kern_range)
	/* FALLTHROUGH */

ENTRY(v6_coherent_user_range)
 UNWIND(.fnstart		)
#ifdef HARVARD_CACHE
	bic	r0, r0, #CACHE_LINE_SIZE - 1
1:
 USER(	mcr	p15, 0, r0, c7, c10, 1	)	@ clean D line
	add	r0, r0, #CACHE_LINE_SIZE
2:
	cmp	r0, r1
	blo	1b
#endif
	mov	r0, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
#ifndef CONFIG_ARM_ERRATA_411920
	mcr	p15, 0, r0, c7, c5, 0		@ I+BTB cache invalidate
#else
	b	v6_icache_inval_all
#endif
#else
	mcr	p15, 0, r0, c7, c5, 6		@ invalidate BTB
#endif
	mov	pc, lr

9001:
	mov	r0, r0, lsr #12
	mov	r0, r0, lsl #12
	add	r0, r0, #4096
	b	2b
 UNWIND(.fnend		)
ENDPROC(v6_coherent_user_range)
ENDPROC(v6_coherent_kern_range)

ENTRY(v6_flush_kern_dcache_area)
	add	r1, r0, r1
1:
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c14, 1		@ clean & invalidate D line
#else
	mcr	p15, 0, r0, c7, c15, 1		@ clean & invalidate unified line
#endif	
	add	r0, r0, #D_CACHE_LINE_SIZE
	cmp	r0, r1
	blo	1b
#ifdef HARVARD_CACHE
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4
#endif
	mov	pc, lr


v6_dma_inv_range:
	tst	r0, #D_CACHE_LINE_SIZE - 1
	bic	r0, r0, #D_CACHE_LINE_SIZE - 1
#ifdef HARVARD_CACHE
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D line
#else
	mcrne	p15, 0, r0, c7, c11, 1		@ clean unified line
#endif
	tst	r1, #D_CACHE_LINE_SIZE - 1
	bic	r1, r1, #D_CACHE_LINE_SIZE - 1
#ifdef HARVARD_CACHE
	mcrne	p15, 0, r1, c7, c14, 1		@ clean & invalidate D line
#else
	mcrne	p15, 0, r1, c7, c15, 1		@ clean & invalidate unified line
#endif
1:
#ifdef CONFIG_DMA_CACHE_RWFO
	ldr	r2, [r0]			@ read for ownership
	str	r2, [r0]			@ write for ownership
#endif
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D line
#else
	mcr	p15, 0, r0, c7, c7, 1		@ invalidate unified line
#endif
	add	r0, r0, #D_CACHE_LINE_SIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

v6_dma_clean_range:
	bic	r0, r0, #D_CACHE_LINE_SIZE - 1
1:
#ifdef CONFIG_DMA_CACHE_RWFO
	ldr	r2, [r0]			@ read for ownership
#endif
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c10, 1		@ clean D line
#else
	mcr	p15, 0, r0, c7, c11, 1		@ clean unified line
#endif
	add	r0, r0, #D_CACHE_LINE_SIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

ENTRY(v6_dma_flush_range)
#ifdef CONFIG_CACHE_FLUSH_RANGE_LIMIT
	sub	r2, r1, r0
	cmp	r2, #CONFIG_CACHE_FLUSH_RANGE_LIMIT
	bhi	v6_dma_flush_dcache_all
#endif
	bic	r0, r0, #D_CACHE_LINE_SIZE - 1
1:
#ifdef CONFIG_DMA_CACHE_RWFO
	ldr	r2, [r0]			@ read for ownership
	str	r2, [r0]			@ write for ownership
#endif
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c14, 1		@ clean & invalidate D line
#else
	mcr	p15, 0, r0, c7, c15, 1		@ clean & invalidate line
#endif
	add	r0, r0, #D_CACHE_LINE_SIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr

#ifdef CONFIG_CACHE_FLUSH_RANGE_LIMIT
v6_dma_flush_dcache_all:
	mov	r0, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r0, c7, c14, 0		@ D cache clean+invalidate
#else
	mcr	p15, 0, r0, c7, c15, 0		@ Cache clean+invalidate
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mov	pc, lr
#endif

ENTRY(v6_dma_map_area)
	add	r1, r1, r0
	teq	r2, #DMA_FROM_DEVICE
	beq	v6_dma_inv_range
#ifndef CONFIG_DMA_CACHE_RWFO
	b	v6_dma_clean_range
#else
	teq	r2, #DMA_TO_DEVICE
	beq	v6_dma_clean_range
	b	v6_dma_flush_range
#endif
ENDPROC(v6_dma_map_area)

ENTRY(v6_dma_unmap_area)
#ifndef CONFIG_DMA_CACHE_RWFO
	add	r1, r1, r0
	teq	r2, #DMA_TO_DEVICE
	bne	v6_dma_inv_range
#endif
	mov	pc, lr
ENDPROC(v6_dma_unmap_area)

	__INITDATA

	.type	v6_cache_fns, #object
ENTRY(v6_cache_fns)
	.long	v6_flush_kern_cache_all
	.long	v6_flush_user_cache_all
	.long	v6_flush_user_cache_range
	.long	v6_coherent_kern_range
	.long	v6_coherent_user_range
	.long	v6_flush_kern_dcache_area
	.long	v6_dma_map_area
	.long	v6_dma_unmap_area
	.long	v6_dma_flush_range
	.size	v6_cache_fns, . - v6_cache_fns
