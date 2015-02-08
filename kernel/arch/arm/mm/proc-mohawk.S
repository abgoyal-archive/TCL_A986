

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

#define CACHE_DLIMIT	32768

#define CACHE_DLINESIZE	32

ENTRY(cpu_mohawk_proc_init)
	mov	pc, lr

ENTRY(cpu_mohawk_proc_fin)
	stmfd	sp!, {lr}
	mov	ip, #PSR_F_BIT | PSR_I_BIT | SVC_MODE
	msr	cpsr_c, ip
	bl	mohawk_flush_kern_cache_all
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1800			@ ...iz...........
	bic	r0, r0, #0x0006			@ .............ca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}

	.align	5
ENTRY(cpu_mohawk_reset)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I,D caches
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	mrc	p15, 0, ip, c1, c0, 0		@ ctrl register
	bic	ip, ip, #0x0007			@ .............cam
	bic	ip, ip, #0x1100			@ ...i...s........
	mcr	p15, 0, ip, c1, c0, 0		@ ctrl register
	mov	pc, r0

	.align	5
ENTRY(cpu_mohawk_do_idle)
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c7, c0, 4		@ wait for interrupt
	mov	pc, lr

ENTRY(mohawk_flush_user_cache_all)
	/* FALLTHROUGH */

ENTRY(mohawk_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
	mcr	p15, 0, ip, c7, c14, 0		@ clean & invalidate all D cache
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcrne	p15, 0, ip, c7, c10, 0		@ drain write buffer
	mov	pc, lr

ENTRY(mohawk_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT
	bgt	__flush_whole_cache
1:	tst	r2, #VM_EXEC
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c10, 4		@ drain WB
	mov	pc, lr

ENTRY(mohawk_coherent_kern_range)
	/* FALLTHROUGH */

ENTRY(mohawk_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

ENTRY(mohawk_flush_kern_dcache_area)
	add	r1, r0, r1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

mohawk_dma_inv_range:
	tst	r0, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
	tst	r1, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean D entry
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

mohawk_dma_clean_range:
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

ENTRY(mohawk_dma_flush_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:
	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

ENTRY(mohawk_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	mohawk_dma_clean_range
	bcs	mohawk_dma_inv_range
	b	mohawk_dma_flush_range
ENDPROC(mohawk_dma_map_area)

ENTRY(mohawk_dma_unmap_area)
	mov	pc, lr
ENDPROC(mohawk_dma_unmap_area)

ENTRY(mohawk_cache_fns)
	.long	mohawk_flush_kern_cache_all
	.long	mohawk_flush_user_cache_all
	.long	mohawk_flush_user_cache_range
	.long	mohawk_coherent_kern_range
	.long	mohawk_coherent_user_range
	.long	mohawk_flush_kern_dcache_area
	.long	mohawk_dma_map_area
	.long	mohawk_dma_unmap_area
	.long	mohawk_dma_flush_range

ENTRY(cpu_mohawk_dcache_clean_area)
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	subs	r1, r1, #CACHE_DLINESIZE
	bhi	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

	.align	5
ENTRY(cpu_mohawk_switch_mm)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c14, 0		@ clean & invalidate all D cache
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	orr	r0, r0, #0x18			@ cache the page table in L2
	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
	mov	pc, lr

	.align	5
ENTRY(cpu_mohawk_set_pte_ext)
	armv3_set_pte_ext
	mov	r0, r0
	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

	__INIT

	.type	__mohawk_setup, #function
__mohawk_setup:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		@ invalidate I,D caches
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c8, c7		@ invalidate I,D TLBs
	orr	r4, r4, #0x18			@ cache the page table in L2
	mcr	p15, 0, r4, c2, c0, 0		@ load page table pointer

	mov	r0, #0				@ don't allow CP access
	mcr	p15, 0, r0, c15, c1, 0		@ write CP access register

	adr	r5, mohawk_crval
	ldmia	r5, {r5, r6}
	mrc	p15, 0, r0, c1, c0		@ get control register
	bic	r0, r0, r5
	orr	r0, r0, r6
	mov	pc, lr

	.size	__mohawk_setup, . - __mohawk_setup

	/*
	 *  R
	 * .RVI ZFRS BLDP WCAM
	 * .011 1001 ..00 0101
	 *
	 */
	.type	mohawk_crval, #object
mohawk_crval:
	crval	clear=0x00007f3f, mmuset=0x00003905, ucset=0x00001134

	__INITDATA

	.type	mohawk_processor_functions, #object
mohawk_processor_functions:
	.word	v5t_early_abort
	.word	legacy_pabort
	.word	cpu_mohawk_proc_init
	.word	cpu_mohawk_proc_fin
	.word	cpu_mohawk_reset
	.word	cpu_mohawk_do_idle
	.word	cpu_mohawk_dcache_clean_area
	.word	cpu_mohawk_switch_mm
	.word	cpu_mohawk_set_pte_ext
	.size	mohawk_processor_functions, . - mohawk_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5te"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_mohawk_name, #object
cpu_mohawk_name:
	.asciz	"Marvell 88SV331x"
	.size	cpu_mohawk_name, . - cpu_mohawk_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

	.type	__88sv331x_proc_info,#object
__88sv331x_proc_info:
	.long	0x56158000			@ Marvell 88SV331x (MOHAWK)
	.long	0xfffff000
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__mohawk_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_mohawk_name
	.long	mohawk_processor_functions
	.long	v4wbi_tlb_fns
	.long	v4wb_user_fns
	.long	mohawk_cache_fns
	.size	__88sv331x_proc_info, . - __88sv331x_proc_info
