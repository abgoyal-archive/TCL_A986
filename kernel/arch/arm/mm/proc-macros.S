
#include <asm/asm-offsets.h>
#include <asm/thread_info.h>

	.macro	vma_vm_mm, rd, rn
	ldr	\rd, [\rn, #VMA_VM_MM]
	.endm

	.macro	vma_vm_flags, rd, rn
	ldr	\rd, [\rn, #VMA_VM_FLAGS]
	.endm

	.macro	tsk_mm, rd, rn
	ldr	\rd, [\rn, #TI_TASK]
	ldr	\rd, [\rd, #TSK_ACTIVE_MM]
	.endm

	.macro	act_mm, rd
	bic	\rd, sp, #8128
	bic	\rd, \rd, #63
	ldr	\rd, [\rd, #TI_TASK]
	ldr	\rd, [\rd, #TSK_ACTIVE_MM]
	.endm

	.macro	mmid, rd, rn
	ldr	\rd, [\rn, #MM_CONTEXT_ID]
	.endm

	.macro	asid, rd, rn
	and	\rd, \rn, #255
	.endm

	.macro	crval, clear, mmuset, ucset
#ifdef CONFIG_MMU
	.word	\clear
	.word	\mmuset
#else
	.word	\clear
	.word	\ucset
#endif
	.endm

	.macro	dcache_line_size, reg, tmp
	mrc	p15, 1, \tmp, c0, c0, 0		@ read CSIDR
	and	\tmp, \tmp, #7			@ cache line size encoding
	mov	\reg, #16			@ size offset
	mov	\reg, \reg, lsl \tmp		@ actual cache line size
	.endm


#ifdef CONFIG_MMU
#if L_PTE_SHARED != PTE_EXT_SHARED
#error PTE shared bit mismatch
#endif
#if (L_PTE_EXEC+L_PTE_USER+L_PTE_WRITE+L_PTE_DIRTY+L_PTE_YOUNG+\
     L_PTE_FILE+L_PTE_PRESENT) > L_PTE_SHARED
#error Invalid Linux PTE bit settings
#endif
#endif	/* CONFIG_MMU */

	.macro	armv6_mt_table pfx
\pfx\()_mt_table:
	.long	0x00						@ L_PTE_MT_UNCACHED
	.long	PTE_EXT_TEX(1)					@ L_PTE_MT_BUFFERABLE
	.long	PTE_CACHEABLE					@ L_PTE_MT_WRITETHROUGH
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_WRITEBACK
	.long	PTE_BUFFERABLE					@ L_PTE_MT_DEV_SHARED
	.long	0x00						@ unused
	.long	0x00						@ L_PTE_MT_MINICACHE (not present)
	.long	PTE_EXT_TEX(1) | PTE_CACHEABLE | PTE_BUFFERABLE	@ L_PTE_MT_WRITEALLOC
	.long	0x00						@ unused
	.long	PTE_EXT_TEX(1)					@ L_PTE_MT_DEV_WC
	.long	0x00						@ unused
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_DEV_CACHED
	.long	PTE_EXT_TEX(2)					@ L_PTE_MT_DEV_NONSHARED
	.long	0x00						@ unused
	.long	0x00						@ unused
	.long	0x00						@ unused
	.endm

	.macro	armv6_set_pte_ext pfx
	str	r1, [r0], #-2048		@ linux version

	bic	r3, r1, #0x000003fc
	bic	r3, r3, #PTE_TYPE_MASK
	orr	r3, r3, r2
	orr	r3, r3, #PTE_EXT_AP0 | 2

	adr	ip, \pfx\()_mt_table
	and	r2, r1, #L_PTE_MT_MASK
	ldr	r2, [ip, r2]

	tst	r1, #L_PTE_WRITE
	tstne	r1, #L_PTE_DIRTY
	orreq	r3, r3, #PTE_EXT_APX

	tst	r1, #L_PTE_USER
	orrne	r3, r3, #PTE_EXT_AP1
	tstne	r3, #PTE_EXT_APX
	bicne	r3, r3, #PTE_EXT_APX | PTE_EXT_AP0

	tst	r1, #L_PTE_EXEC
	orreq	r3, r3, #PTE_EXT_XN

	orr	r3, r3, r2

	tst	r1, #L_PTE_YOUNG
	tstne	r1, #L_PTE_PRESENT
	moveq	r3, #0

	str	r3, [r0]
	mcr	p15, 0, r0, c7, c10, 1		@ flush_pte
	.endm


	.macro	armv3_set_pte_ext wc_disable=1
	str	r1, [r0], #-2048		@ linux version

	eor	r3, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	bic	r2, r1, #PTE_SMALL_AP_MASK	@ keep C, B bits
	bic	r2, r2, #PTE_TYPE_MASK
	orr	r2, r2, #PTE_TYPE_SMALL

	tst	r3, #L_PTE_USER			@ user?
	orrne	r2, r2, #PTE_SMALL_AP_URO_SRW

	tst	r3, #L_PTE_WRITE | L_PTE_DIRTY	@ write and dirty?
	orreq	r2, r2, #PTE_SMALL_AP_UNO_SRW

	tst	r3, #L_PTE_PRESENT | L_PTE_YOUNG	@ present and young?
	movne	r2, #0

	.if	\wc_disable
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	tst	r2, #PTE_CACHEABLE
	bicne	r2, r2, #PTE_BUFFERABLE
#endif
	.endif
	str	r2, [r0]			@ hardware version
	.endm


	.macro	xscale_set_pte_ext_prologue
	str	r1, [r0], #-2048		@ linux version

	eor	r3, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	bic	r2, r1, #PTE_SMALL_AP_MASK	@ keep C, B bits
	orr	r2, r2, #PTE_TYPE_EXT		@ extended page

	tst	r3, #L_PTE_USER			@ user?
	orrne	r2, r2, #PTE_EXT_AP_URO_SRW	@ yes -> user r/o, system r/w

	tst	r3, #L_PTE_WRITE | L_PTE_DIRTY	@ write and dirty?
	orreq	r2, r2, #PTE_EXT_AP_UNO_SRW	@ yes -> user n/a, system r/w
						@ combined with user -> user r/w
	.endm

	.macro	xscale_set_pte_ext_epilogue
	tst	r3, #L_PTE_PRESENT | L_PTE_YOUNG	@ present and young?
	movne	r2, #0				@ no -> fault

	str	r2, [r0]			@ hardware version
	mov	ip, #0
	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
	.endm
