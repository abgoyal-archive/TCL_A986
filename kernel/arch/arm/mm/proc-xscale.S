

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/hwcap.h>
#include <asm/pgtable.h>
#include <asm/pgtable-hwdef.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

#define MAX_AREA_SIZE	32768

#define CACHELINESIZE	32

#define CACHESIZE	32768

#define CLEAN_ADDR	0xfffe0000

	.macro	cpwait, rd
	mrc	p15, 0, \rd, c2, c0, 0		@ arbitrary read of cp15
	mov	\rd, \rd			@ wait for completion
	sub 	pc, pc, #4			@ flush instruction pipeline
	.endm

	.macro	cpwait_ret, lr, rd
	mrc	p15, 0, \rd, c2, c0, 0		@ arbitrary read of cp15
	sub	pc, \lr, \rd, LSR #32		@ wait for completion and
						@ flush instruction pipeline
	.endm

	.macro  clean_d_cache, rd, rs
	ldr	\rs, =clean_addr
	ldr	\rd, [\rs]
	eor	\rd, \rd, #CACHESIZE
	str	\rd, [\rs]
	add	\rs, \rd, #CACHESIZE
1:	mcr	p15, 0, \rd, c7, c2, 5		@ allocate D cache line
	add	\rd, \rd, #CACHELINESIZE
	mcr	p15, 0, \rd, c7, c2, 5		@ allocate D cache line
	add	\rd, \rd, #CACHELINESIZE
	mcr	p15, 0, \rd, c7, c2, 5		@ allocate D cache line
	add	\rd, \rd, #CACHELINESIZE
	mcr	p15, 0, \rd, c7, c2, 5		@ allocate D cache line
	add	\rd, \rd, #CACHELINESIZE
	teq	\rd, \rs
	bne	1b
	.endm

	.data
clean_addr:	.word	CLEAN_ADDR

	.text

ENTRY(cpu_xscale_proc_init)
	@ enable write buffer coalescing. Some bootloader disable it
	mrc	p15, 0, r1, c1, c0, 1
	bic	r1, r1, #1
	mcr	p15, 0, r1, c1, c0, 1
	mov	pc, lr

ENTRY(cpu_xscale_proc_fin)
	str	lr, [sp, #-4]!
	mov	r0, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r0
	bl	xscale_flush_kern_cache_all	@ clean caches
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1800			@ ...IZ...........
	bic	r0, r0, #0x0006			@ .............CA.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldr	pc, [sp], #4

	.align	5
ENTRY(cpu_xscale_reset)
	mov	r1, #PSR_F_BIT|PSR_I_BIT|SVC_MODE
	msr	cpsr_c, r1			@ reset CPSR
	mcr	p15, 0, r1, c10, c4, 1		@ unlock I-TLB
	mcr	p15, 0, r1, c8, c5, 0		@ invalidate I-TLB
	mrc	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x0086			@ ........B....CA.
	bic	r1, r1, #0x3900			@ ..VIZ..S........
	sub	pc, pc, #4			@ flush pipeline
	@ *** cache line aligned ***
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x0001			@ ...............M
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I,D caches & BTB
	mcr	p15, 0, r1, c1, c0, 0		@ ctrl register
	@ CAUTION: MMU turned off from this point. We count on the pipeline
	@ already containing those two last instructions to survive.
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	mov	pc, r0

	.align	5

ENTRY(cpu_xscale_do_idle)
	mov	r0, #1
	mcr	p14, 0, r0, c7, c0, 0		@ Go to IDLE
	mov	pc, lr

/* ================================= CACHE ================================ */

ENTRY(xscale_flush_user_cache_all)
	/* FALLTHROUGH */

ENTRY(xscale_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
	clean_d_cache r0, r1
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ Invalidate I cache & BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

	.align	5
ENTRY(xscale_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #MAX_AREA_SIZE
	bhs	__flush_whole_cache

1:	tst	r2, #VM_EXEC
	mcrne	p15, 0, r0, c7, c5, 1		@ Invalidate I cache line
	mcr	p15, 0, r0, c7, c10, 1		@ Clean D cache line
	mcr	p15, 0, r0, c7, c6, 1		@ Invalidate D cache line
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 6		@ Invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

ENTRY(xscale_coherent_kern_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate I cache & BTB
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

ENTRY(xscale_coherent_user_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c5, 1		@ Invalidate I cache entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

ENTRY(xscale_flush_kern_dcache_area)
	add	r1, r0, r1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ Invalidate I cache & BTB
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

xscale_dma_inv_range:
	tst	r0, #CACHELINESIZE - 1
	bic	r0, r0, #CACHELINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
	tst	r1, #CACHELINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean D entry
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

xscale_dma_clean_range:
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

ENTRY(xscale_dma_flush_range)
	bic	r0, r0, #CACHELINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHELINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mov	pc, lr

ENTRY(xscale_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	xscale_dma_clean_range
	bcs	xscale_dma_inv_range
	b	xscale_dma_flush_range
ENDPROC(xscale_dma_map_area)

ENTRY(xscale_dma_a0_map_area)
	add	r1, r1, r0
	teq	r2, #DMA_TO_DEVICE
	beq	xscale_dma_clean_range
	b	xscale_dma_flush_range
ENDPROC(xscsale_dma_a0_map_area)

ENTRY(xscale_dma_unmap_area)
	mov	pc, lr
ENDPROC(xscale_dma_unmap_area)

ENTRY(xscale_cache_fns)
	.long	xscale_flush_kern_cache_all
	.long	xscale_flush_user_cache_all
	.long	xscale_flush_user_cache_range
	.long	xscale_coherent_kern_range
	.long	xscale_coherent_user_range
	.long	xscale_flush_kern_dcache_area
	.long	xscale_dma_map_area
	.long	xscale_dma_unmap_area
	.long	xscale_dma_flush_range

ENTRY(xscale_80200_A0_A1_cache_fns)
	.long	xscale_flush_kern_cache_all
	.long	xscale_flush_user_cache_all
	.long	xscale_flush_user_cache_range
	.long	xscale_coherent_kern_range
	.long	xscale_coherent_user_range
	.long	xscale_flush_kern_dcache_area
	.long	xscale_dma_a0_map_area
	.long	xscale_dma_unmap_area
	.long	xscale_dma_flush_range

ENTRY(cpu_xscale_dcache_clean_area)
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHELINESIZE
	subs	r1, r1, #CACHELINESIZE
	bhi	1b
	mov	pc, lr

/* =============================== PageTable ============================== */

	.align	5
ENTRY(cpu_xscale_switch_mm)
	clean_d_cache r1, r2
	mcr	p15, 0, ip, c7, c5, 0		@ Invalidate I cache & BTB
	mcr	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	cpwait_ret lr, ip

cpu_xscale_mt_table:
	.long	0x00						@ L_PTE_MT_UNCACHED
	.long	PTE_BUFFERABLE					@ L_PTE_MT_BUFFERABLE
	.long	PTE_CACHEABLE					@ L_PTE_MT_WRITETHROUGH
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_WRITEBACK
	.long	PTE_EXT_TEX(1) | PTE_BUFFERABLE			@ L_PTE_MT_DEV_SHARED
	.long	0x00						@ unused
	.long	PTE_EXT_TEX(1) | PTE_CACHEABLE			@ L_PTE_MT_MINICACHE
	.long	PTE_EXT_TEX(1) | PTE_CACHEABLE | PTE_BUFFERABLE	@ L_PTE_MT_WRITEALLOC
	.long	0x00						@ unused
	.long	PTE_BUFFERABLE					@ L_PTE_MT_DEV_WC
	.long	0x00						@ unused
	.long	PTE_CACHEABLE | PTE_BUFFERABLE			@ L_PTE_MT_DEV_CACHED
	.long	0x00						@ L_PTE_MT_DEV_NONSHARED
	.long	0x00						@ unused
	.long	0x00						@ unused
	.long	0x00						@ unused

	.align	5
ENTRY(cpu_xscale_set_pte_ext)
	xscale_set_pte_ext_prologue

	@
	@ Erratum 40: must set memory to write-through for user read-only pages
	@
	and	ip, r1, #(L_PTE_MT_MASK | L_PTE_USER | L_PTE_WRITE) & ~(4 << 2)
	teq	ip, #L_PTE_MT_WRITEBACK | L_PTE_USER

	moveq	r1, #L_PTE_MT_WRITETHROUGH
	and	r1, r1, #L_PTE_MT_MASK
	adr	ip, cpu_xscale_mt_table
	ldr	ip, [ip, r1]
	bic	r2, r2, #0x0c
	orr	r2, r2, ip

	xscale_set_pte_ext_epilogue
	mov	pc, lr


	.ltorg

	.align

	__INIT

	.type	__xscale_setup, #function
__xscale_setup:
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I, D caches & BTB
	mcr	p15, 0, ip, c7, c10, 4		@ Drain Write (& Fill) Buffer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I, D TLBs
	mov	r0, #1 << 6			@ cp6 for IOP3xx and Bulverde
	orr	r0, r0, #1 << 13		@ Its undefined whether this
	mcr	p15, 0, r0, c15, c1, 0		@ affects USR or SVC modes

	adr	r5, xscale_crval
	ldmia	r5, {r5, r6}
	mrc	p15, 0, r0, c1, c0, 0		@ get control register
	bic	r0, r0, r5
	orr	r0, r0, r6
	mov	pc, lr
	.size	__xscale_setup, . - __xscale_setup

	/*
	 *  R
	 * .RVI ZFRS BLDP WCAM
	 * ..11 1.01 .... .101
	 * 
	 */
	.type	xscale_crval, #object
xscale_crval:
	crval	clear=0x00003b07, mmuset=0x00003905, ucset=0x00001900

	__INITDATA


	.type	xscale_processor_functions, #object
ENTRY(xscale_processor_functions)
	.word	v5t_early_abort
	.word	legacy_pabort
	.word	cpu_xscale_proc_init
	.word	cpu_xscale_proc_fin
	.word	cpu_xscale_reset
	.word	cpu_xscale_do_idle
	.word	cpu_xscale_dcache_clean_area
	.word	cpu_xscale_switch_mm
	.word	cpu_xscale_set_pte_ext
	.size	xscale_processor_functions, . - xscale_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5te"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_80200_A0_A1_name, #object
cpu_80200_A0_A1_name:
	.asciz	"XScale-80200 A0/A1"
	.size	cpu_80200_A0_A1_name, . - cpu_80200_A0_A1_name

	.type	cpu_80200_name, #object
cpu_80200_name:
	.asciz	"XScale-80200"
	.size	cpu_80200_name, . - cpu_80200_name

	.type	cpu_80219_name, #object
cpu_80219_name:
	.asciz	"XScale-80219"
	.size	cpu_80219_name, . - cpu_80219_name

	.type	cpu_8032x_name, #object
cpu_8032x_name:
	.asciz	"XScale-IOP8032x Family"
	.size	cpu_8032x_name, . - cpu_8032x_name

	.type	cpu_8033x_name, #object
cpu_8033x_name:
	.asciz	"XScale-IOP8033x Family"
	.size	cpu_8033x_name, . - cpu_8033x_name

	.type	cpu_pxa250_name, #object
cpu_pxa250_name:
	.asciz	"XScale-PXA250"
	.size	cpu_pxa250_name, . - cpu_pxa250_name

	.type	cpu_pxa210_name, #object
cpu_pxa210_name:
	.asciz	"XScale-PXA210"
	.size	cpu_pxa210_name, . - cpu_pxa210_name

	.type	cpu_ixp42x_name, #object
cpu_ixp42x_name:
	.asciz	"XScale-IXP42x Family"
	.size	cpu_ixp42x_name, . - cpu_ixp42x_name

	.type	cpu_ixp43x_name, #object
cpu_ixp43x_name:
	.asciz	"XScale-IXP43x Family"
	.size	cpu_ixp43x_name, . - cpu_ixp43x_name

	.type	cpu_ixp46x_name, #object
cpu_ixp46x_name:
	.asciz	"XScale-IXP46x Family"
	.size	cpu_ixp46x_name, . - cpu_ixp46x_name

	.type	cpu_ixp2400_name, #object
cpu_ixp2400_name:
	.asciz	"XScale-IXP2400"
	.size	cpu_ixp2400_name, . - cpu_ixp2400_name

	.type	cpu_ixp2800_name, #object
cpu_ixp2800_name:
	.asciz	"XScale-IXP2800"
	.size	cpu_ixp2800_name, . - cpu_ixp2800_name

	.type	cpu_pxa255_name, #object
cpu_pxa255_name:
	.asciz	"XScale-PXA255"
	.size	cpu_pxa255_name, . - cpu_pxa255_name

	.type	cpu_pxa270_name, #object
cpu_pxa270_name:
	.asciz	"XScale-PXA270"
	.size	cpu_pxa270_name, . - cpu_pxa270_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

	.type	__80200_A0_A1_proc_info,#object
__80200_A0_A1_proc_info:
	.long	0x69052000
	.long	0xfffffffe
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_80200_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_80200_A0_A1_cache_fns
	.size	__80200_A0_A1_proc_info, . - __80200_A0_A1_proc_info

	.type	__80200_proc_info,#object
__80200_proc_info:
	.long	0x69052000
	.long	0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_80200_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__80200_proc_info, . - __80200_proc_info

	.type	__80219_proc_info,#object
__80219_proc_info:
	.long	0x69052e20
	.long	0xffffffe0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_80219_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__80219_proc_info, . - __80219_proc_info

	.type	__8032x_proc_info,#object
__8032x_proc_info:
	.long	0x69052420
	.long	0xfffff7e0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_8032x_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__8032x_proc_info, . - __8032x_proc_info

	.type	__8033x_proc_info,#object
__8033x_proc_info:
	.long	0x69054010
	.long	0xfffffd30
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_8033x_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__8033x_proc_info, . - __8033x_proc_info

	.type	__pxa250_proc_info,#object
__pxa250_proc_info:
	.long	0x69052100
	.long	0xfffff7f0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_pxa250_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__pxa250_proc_info, . - __pxa250_proc_info

	.type	__pxa210_proc_info,#object
__pxa210_proc_info:
	.long	0x69052120
	.long	0xfffff3f0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_pxa210_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__pxa210_proc_info, . - __pxa210_proc_info

	.type	__ixp2400_proc_info, #object
__ixp2400_proc_info:
	.long   0x69054190
	.long   0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b       __xscale_setup
	.long   cpu_arch_name
	.long   cpu_elf_name
	.long   HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long   cpu_ixp2400_name
	.long   xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size   __ixp2400_proc_info, . - __ixp2400_proc_info                

	.type	__ixp2800_proc_info, #object
__ixp2800_proc_info:
	.long   0x690541a0
	.long   0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b       __xscale_setup
	.long   cpu_arch_name
	.long   cpu_elf_name
	.long   HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long   cpu_ixp2800_name
	.long   xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size   __ixp2800_proc_info, . - __ixp2800_proc_info                

	.type	__ixp42x_proc_info, #object
__ixp42x_proc_info:
	.long   0x690541c0
	.long   0xffffffc0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b       __xscale_setup
	.long   cpu_arch_name
	.long   cpu_elf_name
	.long   HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long   cpu_ixp42x_name
	.long   xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size   __ixp42x_proc_info, . - __ixp42x_proc_info                

	.type   __ixp43x_proc_info, #object
__ixp43x_proc_info:
	.long   0x69054040
	.long   0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b       __xscale_setup
	.long   cpu_arch_name
	.long   cpu_elf_name
	.long   HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long   cpu_ixp43x_name
	.long   xscale_processor_functions
	.long   v4wbi_tlb_fns
	.long   xscale_mc_user_fns
	.long   xscale_cache_fns
	.size   __ixp43x_proc_info, . - __ixp43x_proc_info

	.type	__ixp46x_proc_info, #object
__ixp46x_proc_info:
	.long   0x69054200
	.long   0xffffff00
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b       __xscale_setup
	.long   cpu_arch_name
	.long   cpu_elf_name
	.long   HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long   cpu_ixp46x_name
	.long   xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size   __ixp46x_proc_info, . - __ixp46x_proc_info

	.type	__pxa255_proc_info,#object
__pxa255_proc_info:
	.long	0x69052d00
	.long	0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_pxa255_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__pxa255_proc_info, . - __pxa255_proc_info

	.type	__pxa270_proc_info,#object
__pxa270_proc_info:
	.long	0x69054110
	.long	0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__xscale_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_pxa270_name
	.long	xscale_processor_functions
	.long	v4wbi_tlb_fns
	.long	xscale_mc_user_fns
	.long	xscale_cache_fns
	.size	__pxa270_proc_info, . - __pxa270_proc_info

