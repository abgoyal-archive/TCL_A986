

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/hwcap.h>
#include <mach/hardware.h>
#include <asm/pgtable.h>
#include <asm/pgtable-hwdef.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

#define MAX_AREA_SIZE	32768

#define CACHELINESIZE	32

#define CACHESIZE	32768

	.macro	cpwait_ret, lr, rd
	mrc	p15, 0, \rd, c2, c0, 0		@ arbitrary read of cp15
	sub	pc, \lr, \rd, LSR #32		@ wait for completion and
						@ flush instruction pipeline
	.endm


 	.macro  clean_d_cache rd, rs
	mov	\rd, #0x1f00
	orr	\rd, \rd, #0x00e0
1:	mcr	p15, 0, \rd, c7, c14, 2		@ clean/invalidate L1 D line
	adds	\rd, \rd, #0x40000000
	bcc	1b
	subs	\rd, \rd, #0x20
	bpl	1b
	.endm

	.text

ENTRY(cpu_xsc3_proc_init)
	mov	pc, lr

ENTRY(cpu_xsc3_proc_fin)
	str	lr, [sp, #-4]!
	mov	r0, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r0
	bl	xsc3_flush_kern_cache_all	@ clean caches
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1800			@ ...IZ...........
	bic	r0, r0, #0x0006			@ .............CA.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldr	pc, [sp], #4

	.align	5
ENTRY(cpu_xsc3_reset)
	mov	r1, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r1			@ reset CPSR
	mrc	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x3900			@ ..VIZ..S........
	bic	r1, r1, #0x0086			@ ........B....CA.
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate L1 caches and BTB
	bic	r1, r1, #0x0001			@ ...............M
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	@ CAUTION: MMU turned off from this point.  We count on the pipeline
	@ already containing those two last instructions to survive.
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	mov	pc, r0

	.align	5

ENTRY(cpu_xsc3_do_idle)
	mov	r0, #1
	mcr	p14, 0, r0, c7, c0, 0		@ go to idle
	mov	pc, lr

/* ================================= CACHE ================================ */

ENTRY(xsc3_flush_user_cache_all)
	/* FALLTHROUGH */

ENTRY(xsc3_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
	clean_d_cache r0, r1
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ data write barrier
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
	mov	pc, lr

	.align	5
ENTRY(xsc3_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #MAX_AREA_SIZE
	bhs	__flush_whole_cache

1:	tst	r2, #VM_EXEC
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate L1 I line
	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 6		@ invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ data write barrier
	mcrne	p15, 0, ip, c7, c5, 4		@ prefetch flush
	mov	pc, lr

ENTRY(xsc3_coherent_kern_range)
/* FALLTHROUGH */
ENTRY(xsc3_coherent_user_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
	mov	pc, lr

ENTRY(xsc3_flush_kern_dcache_area)
	add	r1, r0, r1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
	mov	pc, lr

xsc3_dma_inv_range:
	tst	r0, #CACHELINESIZE - 1
	bic	r0, r0, #CACHELINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	tst	r1, #CACHELINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean L1 D line
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	mov	pc, lr

xsc3_dma_clean_range:
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	mov	pc, lr

ENTRY(xsc3_dma_flush_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean/invalidate L1 D line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ data write barrier
	mov	pc, lr

ENTRY(xsc3_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	xsc3_dma_clean_range
	bcs	xsc3_dma_inv_range
	b	xsc3_dma_flush_range
ENDPROC(xsc3_dma_map_area)

ENTRY(xsc3_dma_unmap_area)
	mov	pc, lr
ENDPROC(xsc3_dma_unmap_area)

ENTRY(xsc3_cache_fns)
	.long	xsc3_flush_kern_cache_all
	.long	xsc3_flush_user_cache_all
	.long	xsc3_flush_user_cache_range
	.long	xsc3_coherent_kern_range
	.long	xsc3_coherent_user_range
	.long	xsc3_flush_kern_dcache_area
	.long	xsc3_dma_map_area
	.long	xsc3_dma_unmap_area
	.long	xsc3_dma_flush_range

ENTRY(cpu_xsc3_dcache_clean_area)
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean L1 D line
	add	r0, r0, #CACHELINESIZE
	subs	r1, r1, #CACHELINESIZE
	bhi	1b
	mov	pc, lr

/* =============================== PageTable ============================== */

	.align	5
ENTRY(cpu_xsc3_switch_mm)
	clean_d_cache r1, r2
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate L1 I cache and BTB
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
	mcr	p15, 0, ip, c7, c5, 4		@ prefetch flush
	orr	r0, r0, #0x18			@ cache the page table in L2
	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	cpwait_ret lr, ip

cpu_xsc3_mt_table:
	.long	0x00						@ L_PTE_MT_UNCACHED
	.long	PTE_EXT_TEX(1)					@ L_PTE_MT_BUFFERABLE
	.long	PTE_EXT_TEX(5) | PTE_CACHEABLE			@ L_PTE_MT_WRITETHROUGH
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_WRITEBACK
	.long	PTE_EXT_TEX(1) | PTE_BUFFERABLE			@ L_PTE_MT_DEV_SHARED
	.long	0x00						@ unused
	.long	0x00						@ L_PTE_MT_MINICACHE (not present)
	.long	PTE_EXT_TEX(5) | PTE_CACHEABLE | PTE_BUFFERABLE	@ L_PTE_MT_WRITEALLOC (not present?)
	.long	0x00						@ unused
	.long	PTE_EXT_TEX(1)					@ L_PTE_MT_DEV_WC
	.long	0x00						@ unused
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_DEV_CACHED
	.long	PTE_EXT_TEX(2)					@ L_PTE_MT_DEV_NONSHARED
	.long	0x00						@ unused
	.long	0x00						@ unused
	.long	0x00						@ unused

	.align	5
ENTRY(cpu_xsc3_set_pte_ext)
	xscale_set_pte_ext_prologue

	tst	r1, #L_PTE_SHARED		@ shared?
	and	r1, r1, #L_PTE_MT_MASK
	adr	ip, cpu_xsc3_mt_table
	ldr	ip, [ip, r1]
	orrne	r2, r2, #PTE_EXT_COHERENT	@ interlock: mask in coherent bit
	bic	r2, r2, #0x0c			@ clear old C,B bits
	orr	r2, r2, ip

	xscale_set_pte_ext_epilogue
	mov	pc, lr

	.ltorg

	.align

	__INIT

	.type	__xsc3_setup, #function
__xsc3_setup:
	mov	r0, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate L1 caches and BTB
	mcr	p15, 0, ip, c7, c10, 4		@ data write barrier
	mcr	p15, 0, ip, c7, c5, 4		@ prefetch flush
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	orr	r4, r4, #0x18			@ cache the page table in L2
	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer

	mov	r0, #1 << 6			@ cp6 access for early sched_clock
	mcr	p15, 0, r0, c15, c1, 0		@ write CP access register

	mrc	p15, 0, r0, c1, c0, 1		@ get auxiliary control reg
	and	r0, r0, #2			@ preserve bit P bit setting
	orr	r0, r0, #(1 << 10)		@ enable L2 for LLR cache
	mcr	p15, 0, r0, c1, c0, 1		@ set auxiliary control reg

	adr	r5, xsc3_crval
	ldmia	r5, {r5, r6}

#ifdef CONFIG_CACHE_XSC3L2
	mrc	p15, 1, r0, c0, c0, 1		@ get L2 present information
	ands	r0, r0, #0xf8
	orrne	r6, r6, #(1 << 26)		@ enable L2 if present
#endif

	mrc	p15, 0, r0, c1, c0, 0		@ get control register
	bic	r0, r0, r5			@ ..V. ..R. .... ..A.
	orr	r0, r0, r6			@ ..VI Z..S .... .C.M (mmu)
						@ ...I Z..S .... .... (uc)
	mov	pc, lr

	.size	__xsc3_setup, . - __xsc3_setup

	.type	xsc3_crval, #object
xsc3_crval:
	crval	clear=0x04002202, mmuset=0x00003905, ucset=0x00001900

	__INITDATA


	.type	xsc3_processor_functions, #object
ENTRY(xsc3_processor_functions)
	.word	v5t_early_abort
	.word	legacy_pabort
	.word	cpu_xsc3_proc_init
	.word	cpu_xsc3_proc_fin
	.word	cpu_xsc3_reset
	.word	cpu_xsc3_do_idle
	.word	cpu_xsc3_dcache_clean_area
	.word	cpu_xsc3_switch_mm
	.word	cpu_xsc3_set_pte_ext
	.size	xsc3_processor_functions, . - xsc3_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5te"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_xsc3_name, #object
cpu_xsc3_name:
	.asciz	"XScale-V3 based processor"
	.size	cpu_xsc3_name, . - cpu_xsc3_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

	.type	__xsc3_proc_info,#object
__xsc3_proc_info:
	.long	0x69056000
	.long	0xffffe000
	.long	PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long	PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xsc3_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_xsc3_name
	.long	xsc3_processor_functions
	.long	v4wbi_tlb_fns
	.long	xsc3_mc_user_fns
	.long	xsc3_cache_fns
	.size	__xsc3_proc_info, . - __xsc3_proc_info

/* Note: PXA935 changed its implementor ID from Intel to Marvell */

	.type	__xsc3_pxa935_proc_info,#object
__xsc3_pxa935_proc_info:
	.long	0x56056000
	.long	0xffffe000
	.long	PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long	PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xsc3_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_xsc3_name
	.long	xsc3_processor_functions
	.long	v4wbi_tlb_fns
	.long	xsc3_mc_user_fns
	.long	xsc3_cache_fns
	.size	__xsc3_pxa935_proc_info, . - __xsc3_pxa935_proc_info
