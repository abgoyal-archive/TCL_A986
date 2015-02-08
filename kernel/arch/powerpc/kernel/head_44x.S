

#include <linux/init.h>
#include <asm/processor.h>
#include <asm/page.h>
#include <asm/mmu.h>
#include <asm/pgtable.h>
#include <asm/cputable.h>
#include <asm/thread_info.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>
#include <asm/synch.h>
#include "head_booke.h"


	__HEAD
_ENTRY(_stext);
_ENTRY(_start);
	/*
	 * Reserve a word at a fixed location to store the address
	 * of abatron_pteptrs
	 */
	nop
	mr	r31,r3
	mr	r30,r4
	mr	r29,r5
	mr	r28,r6
	mr	r27,r7
	li	r24,0		/* CPU number */

	bl	init_cpu_state

	/*
	 * This is where the main kernel code starts.
	 */

	/* ptr to current */
	lis	r2,init_task@h
	ori	r2,r2,init_task@l

	/* ptr to current thread */
	addi	r4,r2,THREAD	/* init task's THREAD */
	mtspr	SPRN_SPRG_THREAD,r4

	/* stack */
	lis	r1,init_thread_union@h
	ori	r1,r1,init_thread_union@l
	li	r0,0
	stwu	r0,THREAD_SIZE-STACK_FRAME_OVERHEAD(r1)

	bl	early_init

	mr	r3,r31
	mr	r4,r30
	mr	r5,r29
	mr	r6,r28
	mr	r7,r27
	bl	machine_init
	bl	MMU_init

	/* Setup PTE pointers for the Abatron bdiGDB */
	lis	r6, swapper_pg_dir@h
	ori	r6, r6, swapper_pg_dir@l
	lis	r5, abatron_pteptrs@h
	ori	r5, r5, abatron_pteptrs@l
	lis	r4, KERNELBASE@h
	ori	r4, r4, KERNELBASE@l
	stw	r5, 0(r4)	/* Save abatron_pteptrs at a fixed location */
	stw	r6, 0(r5)

	/* Let's move on */
	lis	r4,start_kernel@h
	ori	r4,r4,start_kernel@l
	lis	r3,MSR_KERNEL@h
	ori	r3,r3,MSR_KERNEL@l
	mtspr	SPRN_SRR0,r4
	mtspr	SPRN_SRR1,r3
	rfi			/* change context and jump to start_kernel */


interrupt_base:
	/* Critical Input Interrupt */
	CRITICAL_EXCEPTION(0x0100, CriticalInput, unknown_exception)

	/* Machine Check Interrupt */
	CRITICAL_EXCEPTION(0x0200, MachineCheck, machine_check_exception)
	MCHECK_EXCEPTION(0x0210, MachineCheckA, machine_check_exception)

	/* Data Storage Interrupt */
	DATA_STORAGE_EXCEPTION

		/* Instruction Storage Interrupt */
	INSTRUCTION_STORAGE_EXCEPTION

	/* External Input Interrupt */
	EXCEPTION(0x0500, ExternalInput, do_IRQ, EXC_XFER_LITE)

	/* Alignment Interrupt */
	ALIGNMENT_EXCEPTION

	/* Program Interrupt */
	PROGRAM_EXCEPTION

	/* Floating Point Unavailable Interrupt */
#ifdef CONFIG_PPC_FPU
	FP_UNAVAILABLE_EXCEPTION
#else
	EXCEPTION(0x2010, FloatingPointUnavailable, unknown_exception, EXC_XFER_EE)
#endif
	/* System Call Interrupt */
	START_EXCEPTION(SystemCall)
	NORMAL_EXCEPTION_PROLOG
	EXC_XFER_EE_LITE(0x0c00, DoSyscall)

	/* Auxillary Processor Unavailable Interrupt */
	EXCEPTION(0x2020, AuxillaryProcessorUnavailable, unknown_exception, EXC_XFER_EE)

	/* Decrementer Interrupt */
	DECREMENTER_EXCEPTION

	/* Fixed Internal Timer Interrupt */
	/* TODO: Add FIT support */
	EXCEPTION(0x1010, FixedIntervalTimer, unknown_exception, EXC_XFER_EE)

	/* Watchdog Timer Interrupt */
	/* TODO: Add watchdog support */
#ifdef CONFIG_BOOKE_WDT
	CRITICAL_EXCEPTION(0x1020, WatchdogTimer, WatchdogException)
#else
	CRITICAL_EXCEPTION(0x1020, WatchdogTimer, unknown_exception)
#endif

	/* Data TLB Error Interrupt */
	START_EXCEPTION(DataTLBError44x)
	mtspr	SPRN_SPRG_WSCRATCH0, r10		/* Save some working registers */
	mtspr	SPRN_SPRG_WSCRATCH1, r11
	mtspr	SPRN_SPRG_WSCRATCH2, r12
	mtspr	SPRN_SPRG_WSCRATCH3, r13
	mfcr	r11
	mtspr	SPRN_SPRG_WSCRATCH4, r11
	mfspr	r10, SPRN_DEAR		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11, PAGE_OFFSET@h
	cmplw	r10, r11
	blt+	3f
	lis	r11, swapper_pg_dir@h
	ori	r11, r11, swapper_pg_dir@l

	mfspr	r12,SPRN_MMUCR
	rlwinm	r12,r12,0,0,23		/* Clear TID */

	b	4f

	/* Get the PGD for the current thread */
3:
	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)

	/* Load PID into MMUCR TID */
	mfspr	r12,SPRN_MMUCR
	mfspr   r13,SPRN_PID		/* Get PID */
	rlwimi	r12,r13,0,24,31		/* Set TID */

4:
	mtspr	SPRN_MMUCR,r12

	/* Mask of required permission bits. Note that while we
	 * do copy ESR:ST to _PAGE_RW position as trying to write
	 * to an RO page is pretty common, we don't do it with
	 * _PAGE_DIRTY. We could do it, but it's a fairly rare
	 * event so I'd rather take the overhead when it happens
	 * rather than adding an instruction here. We should measure
	 * whether the whole thing is worth it in the first place
	 * as we could avoid loading SPRN_ESR completely in the first
	 * place...
	 *
	 * TODO: Is it worth doing that mfspr & rlwimi in the first
	 *       place or can we save a couple of instructions here ?
	 */
	mfspr	r12,SPRN_ESR
	li	r13,_PAGE_PRESENT|_PAGE_ACCESSED
	rlwimi	r13,r12,10,30,30

	/* Load the PTE */
	/* Compute pgdir/pmd offset */
	rlwinm  r12, r10, PPC44x_PGD_OFF_SHIFT, PPC44x_PGD_OFF_MASK_BIT, 29
	lwzx	r11, r12, r11		/* Get pgd/pmd entry */
	rlwinm.	r12, r11, 0, 0, 20	/* Extract pt base address */
	beq	2f			/* Bail if no table */

	/* Compute pte address */
	rlwimi  r12, r10, PPC44x_PTE_ADD_SHIFT, PPC44x_PTE_ADD_MASK_BIT, 28
	lwz	r11, 0(r12)		/* Get high word of pte entry */
	lwz	r12, 4(r12)		/* Get low word of pte entry */

	lis	r10,tlb_44x_index@ha

	andc.	r13,r13,r12		/* Check permission */

	/* Load the next available TLB index */
	lwz	r13,tlb_44x_index@l(r10)

	bne	2f			/* Bail if permission mismach */

	/* Increment, rollover, and store TLB index */
	addi	r13,r13,1

	/* Compare with watermark (instruction gets patched) */
	.globl tlb_44x_patch_hwater_D
tlb_44x_patch_hwater_D:
	cmpwi	0,r13,1			/* reserve entries */
	ble	5f
	li	r13,0
5:
	/* Store the next available TLB index */
	stw	r13,tlb_44x_index@l(r10)

	/* Re-load the faulting address */
	mfspr	r10,SPRN_DEAR

	 /* Jump to common tlb load */
	b	finish_tlb_load_44x

2:
	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
	mfspr	r11, SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13, SPRN_SPRG_RSCRATCH3
	mfspr	r12, SPRN_SPRG_RSCRATCH2
	mfspr	r11, SPRN_SPRG_RSCRATCH1
	mfspr	r10, SPRN_SPRG_RSCRATCH0
	b	DataStorage

	/* Instruction TLB Error Interrupt */
	/*
	 * Nearly the same as above, except we get our
	 * information from different registers and bailout
	 * to a different point.
	 */
	START_EXCEPTION(InstructionTLBError44x)
	mtspr	SPRN_SPRG_WSCRATCH0, r10 /* Save some working registers */
	mtspr	SPRN_SPRG_WSCRATCH1, r11
	mtspr	SPRN_SPRG_WSCRATCH2, r12
	mtspr	SPRN_SPRG_WSCRATCH3, r13
	mfcr	r11
	mtspr	SPRN_SPRG_WSCRATCH4, r11
	mfspr	r10, SPRN_SRR0		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11, PAGE_OFFSET@h
	cmplw	r10, r11
	blt+	3f
	lis	r11, swapper_pg_dir@h
	ori	r11, r11, swapper_pg_dir@l

	mfspr	r12,SPRN_MMUCR
	rlwinm	r12,r12,0,0,23		/* Clear TID */

	b	4f

	/* Get the PGD for the current thread */
3:
	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)

	/* Load PID into MMUCR TID */
	mfspr	r12,SPRN_MMUCR
	mfspr   r13,SPRN_PID		/* Get PID */
	rlwimi	r12,r13,0,24,31		/* Set TID */

4:
	mtspr	SPRN_MMUCR,r12

	/* Make up the required permissions */
	li	r13,_PAGE_PRESENT | _PAGE_ACCESSED | _PAGE_EXEC

	/* Compute pgdir/pmd offset */
	rlwinm 	r12, r10, PPC44x_PGD_OFF_SHIFT, PPC44x_PGD_OFF_MASK_BIT, 29
	lwzx	r11, r12, r11		/* Get pgd/pmd entry */
	rlwinm.	r12, r11, 0, 0, 20	/* Extract pt base address */
	beq	2f			/* Bail if no table */

	/* Compute pte address */
	rlwimi	r12, r10, PPC44x_PTE_ADD_SHIFT, PPC44x_PTE_ADD_MASK_BIT, 28
	lwz	r11, 0(r12)		/* Get high word of pte entry */
	lwz	r12, 4(r12)		/* Get low word of pte entry */

	lis	r10,tlb_44x_index@ha

	andc.	r13,r13,r12		/* Check permission */

	/* Load the next available TLB index */
	lwz	r13,tlb_44x_index@l(r10)

	bne	2f			/* Bail if permission mismach */

	/* Increment, rollover, and store TLB index */
	addi	r13,r13,1

	/* Compare with watermark (instruction gets patched) */
	.globl tlb_44x_patch_hwater_I
tlb_44x_patch_hwater_I:
	cmpwi	0,r13,1			/* reserve entries */
	ble	5f
	li	r13,0
5:
	/* Store the next available TLB index */
	stw	r13,tlb_44x_index@l(r10)

	/* Re-load the faulting address */
	mfspr	r10,SPRN_SRR0

	/* Jump to common TLB load point */
	b	finish_tlb_load_44x

2:
	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
	mfspr	r11, SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13, SPRN_SPRG_RSCRATCH3
	mfspr	r12, SPRN_SPRG_RSCRATCH2
	mfspr	r11, SPRN_SPRG_RSCRATCH1
	mfspr	r10, SPRN_SPRG_RSCRATCH0
	b	InstructionStorage

finish_tlb_load_44x:
	/* Combine RPN & ERPN an write WS 0 */
	rlwimi	r11,r12,0,0,31-PAGE_SHIFT
	tlbwe	r11,r13,PPC44x_TLB_XLAT

	/*
	 * Create WS1. This is the faulting address (EPN),
	 * page size, and valid flag.
	 */
	li	r11,PPC44x_TLB_VALID | PPC44x_TLBE_SIZE
	/* Insert valid and page size */
	rlwimi	r10,r11,0,PPC44x_PTE_ADD_MASK_BIT,31
	tlbwe	r10,r13,PPC44x_TLB_PAGEID	/* Write PAGEID */

	/* And WS 2 */
	li	r10,0xf85			/* Mask to apply from PTE */
	rlwimi	r10,r12,29,30,30		/* DIRTY -> SW position */
	and	r11,r12,r10			/* Mask PTE bits to keep */
	andi.	r10,r12,_PAGE_USER		/* User page ? */
	beq	1f				/* nope, leave U bits empty */
	rlwimi	r11,r11,3,26,28			/* yes, copy S bits to U */
1:	tlbwe	r11,r13,PPC44x_TLB_ATTRIB	/* Write ATTRIB */

	/* Done...restore registers and get out of here.
	*/
	mfspr	r11, SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13, SPRN_SPRG_RSCRATCH3
	mfspr	r12, SPRN_SPRG_RSCRATCH2
	mfspr	r11, SPRN_SPRG_RSCRATCH1
	mfspr	r10, SPRN_SPRG_RSCRATCH0
	rfi					/* Force context change */

#ifdef CONFIG_PPC_47x
	START_EXCEPTION(DataTLBError47x)
	mtspr	SPRN_SPRG_WSCRATCH0,r10	/* Save some working registers */
	mtspr	SPRN_SPRG_WSCRATCH1,r11
	mtspr	SPRN_SPRG_WSCRATCH2,r12
	mtspr	SPRN_SPRG_WSCRATCH3,r13
	mfcr	r11
	mtspr	SPRN_SPRG_WSCRATCH4,r11
	mfspr	r10,SPRN_DEAR		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11,PAGE_OFFSET@h
	cmplw	cr0,r10,r11
	blt+	3f
	lis	r11,swapper_pg_dir@h
	ori	r11,r11, swapper_pg_dir@l
	li	r12,0			/* MMUCR = 0 */
	b	4f

	/* Get the PGD for the current thread and setup MMUCR */
3:	mfspr	r11,SPRN_SPRG3
	lwz	r11,PGDIR(r11)
	mfspr   r12,SPRN_PID		/* Get PID */
4:	mtspr	SPRN_MMUCR,r12		/* Set MMUCR */

	/* Mask of required permission bits. Note that while we
	 * do copy ESR:ST to _PAGE_RW position as trying to write
	 * to an RO page is pretty common, we don't do it with
	 * _PAGE_DIRTY. We could do it, but it's a fairly rare
	 * event so I'd rather take the overhead when it happens
	 * rather than adding an instruction here. We should measure
	 * whether the whole thing is worth it in the first place
	 * as we could avoid loading SPRN_ESR completely in the first
	 * place...
	 *
	 * TODO: Is it worth doing that mfspr & rlwimi in the first
	 *       place or can we save a couple of instructions here ?
	 */
	mfspr	r12,SPRN_ESR
	li	r13,_PAGE_PRESENT|_PAGE_ACCESSED
	rlwimi	r13,r12,10,30,30

	/* Load the PTE */
	/* Compute pgdir/pmd offset */
	rlwinm  r12,r10,PPC44x_PGD_OFF_SHIFT,PPC44x_PGD_OFF_MASK_BIT,29
	lwzx	r11,r12,r11		/* Get pgd/pmd entry */

	/* Word 0 is EPN,V,TS,DSIZ */
	li	r12,PPC47x_TLB0_VALID | PPC47x_TLBE_SIZE
	rlwimi	r10,r12,0,32-PAGE_SHIFT,31	/* Insert valid and page size*/
	li	r12,0
	tlbwe	r10,r12,0

	/* XXX can we do better ? Need to make sure tlbwe has established
	 * latch V bit in MMUCR0 before the PTE is loaded further down */
#ifdef CONFIG_SMP
	isync
#endif

	rlwinm.	r12,r11,0,0,20		/* Extract pt base address */
	/* Compute pte address */
	rlwimi  r12,r10,PPC44x_PTE_ADD_SHIFT,PPC44x_PTE_ADD_MASK_BIT,28
	beq	2f			/* Bail if no table */
	lwz	r11,0(r12)		/* Get high word of pte entry */

	/* XXX can we do better ? maybe insert a known 0 bit from r11 into the
	 * bottom of r12 to create a data dependency... We can also use r10
	 * as destination nowadays
	 */
#ifdef CONFIG_SMP
	lwsync
#endif
	lwz	r12,4(r12)		/* Get low word of pte entry */

	andc.	r13,r13,r12		/* Check permission */

	 /* Jump to common tlb load */
	beq	finish_tlb_load_47x

2:	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
	mfspr	r11,SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13,SPRN_SPRG_RSCRATCH3
	mfspr	r12,SPRN_SPRG_RSCRATCH2
	mfspr	r11,SPRN_SPRG_RSCRATCH1
	mfspr	r10,SPRN_SPRG_RSCRATCH0
	b	DataStorage

	/* Instruction TLB Error Interrupt */
	/*
	 * Nearly the same as above, except we get our
	 * information from different registers and bailout
	 * to a different point.
	 */
	START_EXCEPTION(InstructionTLBError47x)
	mtspr	SPRN_SPRG_WSCRATCH0,r10	/* Save some working registers */
	mtspr	SPRN_SPRG_WSCRATCH1,r11
	mtspr	SPRN_SPRG_WSCRATCH2,r12
	mtspr	SPRN_SPRG_WSCRATCH3,r13
	mfcr	r11
	mtspr	SPRN_SPRG_WSCRATCH4,r11
	mfspr	r10,SPRN_SRR0		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11,PAGE_OFFSET@h
	cmplw	cr0,r10,r11
	blt+	3f
	lis	r11,swapper_pg_dir@h
	ori	r11,r11, swapper_pg_dir@l
	li	r12,0			/* MMUCR = 0 */
	b	4f

	/* Get the PGD for the current thread and setup MMUCR */
3:	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)
	mfspr   r12,SPRN_PID		/* Get PID */
4:	mtspr	SPRN_MMUCR,r12		/* Set MMUCR */

	/* Make up the required permissions */
	li	r13,_PAGE_PRESENT | _PAGE_ACCESSED | _PAGE_EXEC

	/* Load PTE */
	/* Compute pgdir/pmd offset */
	rlwinm  r12,r10,PPC44x_PGD_OFF_SHIFT,PPC44x_PGD_OFF_MASK_BIT,29
	lwzx	r11,r12,r11		/* Get pgd/pmd entry */

	/* Word 0 is EPN,V,TS,DSIZ */
	li	r12,PPC47x_TLB0_VALID | PPC47x_TLBE_SIZE
	rlwimi	r10,r12,0,32-PAGE_SHIFT,31	/* Insert valid and page size*/
	li	r12,0
	tlbwe	r10,r12,0

	/* XXX can we do better ? Need to make sure tlbwe has established
	 * latch V bit in MMUCR0 before the PTE is loaded further down */
#ifdef CONFIG_SMP
	isync
#endif

	rlwinm.	r12,r11,0,0,20		/* Extract pt base address */
	/* Compute pte address */
	rlwimi  r12,r10,PPC44x_PTE_ADD_SHIFT,PPC44x_PTE_ADD_MASK_BIT,28
	beq	2f			/* Bail if no table */

	lwz	r11,0(r12)		/* Get high word of pte entry */
	/* XXX can we do better ? maybe insert a known 0 bit from r11 into the
	 * bottom of r12 to create a data dependency... We can also use r10
	 * as destination nowadays
	 */
#ifdef CONFIG_SMP
	lwsync
#endif
	lwz	r12,4(r12)		/* Get low word of pte entry */

	andc.	r13,r13,r12		/* Check permission */

	/* Jump to common TLB load point */
	beq	finish_tlb_load_47x

2:	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
	mfspr	r11, SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13, SPRN_SPRG_RSCRATCH3
	mfspr	r12, SPRN_SPRG_RSCRATCH2
	mfspr	r11, SPRN_SPRG_RSCRATCH1
	mfspr	r10, SPRN_SPRG_RSCRATCH0
	b	InstructionStorage

finish_tlb_load_47x:
	/* Combine RPN & ERPN an write WS 1 */
	rlwimi	r11,r12,0,0,31-PAGE_SHIFT
	tlbwe	r11,r13,1

	/* And make up word 2 */
	li	r10,0xf85			/* Mask to apply from PTE */
	rlwimi	r10,r12,29,30,30		/* DIRTY -> SW position */
	and	r11,r12,r10			/* Mask PTE bits to keep */
	andi.	r10,r12,_PAGE_USER		/* User page ? */
	beq	1f				/* nope, leave U bits empty */
	rlwimi	r11,r11,3,26,28			/* yes, copy S bits to U */
1:	tlbwe	r11,r13,2

	/* Done...restore registers and get out of here.
	*/
	mfspr	r11, SPRN_SPRG_RSCRATCH4
	mtcr	r11
	mfspr	r13, SPRN_SPRG_RSCRATCH3
	mfspr	r12, SPRN_SPRG_RSCRATCH2
	mfspr	r11, SPRN_SPRG_RSCRATCH1
	mfspr	r10, SPRN_SPRG_RSCRATCH0
	rfi

#endif /* CONFIG_PPC_47x */

	/* Debug Interrupt */
	/*
	 * This statement needs to exist at the end of the IVPR
	 * definition just in case you end up taking a debug
	 * exception within another exception.
	 */
	DEBUG_CRIT_EXCEPTION


_GLOBAL(__fixup_440A_mcheck)
	li	r3,MachineCheckA@l
	mtspr	SPRN_IVOR1,r3
	sync
	blr

_GLOBAL(giveup_altivec)
	blr

#ifndef CONFIG_PPC_FPU
_GLOBAL(giveup_fpu)
	blr
#endif

_GLOBAL(set_context)

#ifdef CONFIG_BDI_SWITCH
	/* Context switch the PTE pointer for the Abatron BDI2000.
	 * The PGDIR is the second parameter.
	 */
	lis	r5, abatron_pteptrs@h
	ori	r5, r5, abatron_pteptrs@l
	stw	r4, 0x4(r5)
#endif
	mtspr	SPRN_PID,r3
	isync			/* Force context change */
	blr

_GLOBAL(init_cpu_state)
	mflr	r22
#ifdef CONFIG_PPC_47x
	/* We use the PVR to differenciate 44x cores from 476 */
	mfspr	r3,SPRN_PVR
	srwi	r3,r3,16
	cmplwi	cr0,r3,PVR_476@h
	beq	head_start_47x
	cmplwi	cr0,r3,PVR_476_ISS@h
	beq	head_start_47x
#endif /* CONFIG_PPC_47x */

	mfspr	r3,SPRN_CCR0
	rlwinm	r3,r3,0,0,27	/* disable icache prefetch */
	isync
	mtspr	SPRN_CCR0,r3
	isync
	sync


	/* Load our current PID->MMUCR TID and MSR IS->MMUCR STS */
	mfspr	r3,SPRN_PID			/* Get PID */
	mfmsr	r4				/* Get MSR */
	andi.	r4,r4,MSR_IS@l			/* TS=1? */
	beq	wmmucr				/* If not, leave STS=0 */
	oris	r3,r3,PPC44x_MMUCR_STS@h	/* Set STS=1 */
wmmucr:	mtspr	SPRN_MMUCR,r3			/* Put MMUCR */
	sync

	bl	invstr				/* Find our address */
invstr:	mflr	r5				/* Make it accessible */
	tlbsx	r23,0,r5			/* Find entry we are in */
	li	r4,0				/* Start at TLB entry 0 */
	li	r3,0				/* Set PAGEID inval value */
1:	cmpw	r23,r4				/* Is this our entry? */
	beq	skpinv				/* If so, skip the inval */
	tlbwe	r3,r4,PPC44x_TLB_PAGEID		/* If not, inval the entry */
skpinv:	addi	r4,r4,1				/* Increment */
	cmpwi	r4,64				/* Are we done? */
	bne	1b				/* If not, repeat */
	isync					/* If so, context change */


	lis	r3,PAGE_OFFSET@h
	ori	r3,r3,PAGE_OFFSET@l

	/* Kernel is at the base of RAM */
	li r4, 0			/* Load the kernel physical address */

	/* Load the kernel PID = 0 */
	li	r0,0
	mtspr	SPRN_PID,r0
	sync

	/* Initialize MMUCR */
	li	r5,0
	mtspr	SPRN_MMUCR,r5
	sync

	/* pageid fields */
	clrrwi	r3,r3,10		/* Mask off the effective page number */
	ori	r3,r3,PPC44x_TLB_VALID | PPC44x_TLB_256M

	/* xlat fields */
	clrrwi	r4,r4,10		/* Mask off the real page number */
					/* ERPN is 0 for first 4GB page */

	/* attrib fields */
	/* Added guarded bit to protect against speculative loads/stores */
	li	r5,0
	ori	r5,r5,(PPC44x_TLB_SW | PPC44x_TLB_SR | PPC44x_TLB_SX | PPC44x_TLB_G)

        li      r0,63                    /* TLB slot 63 */

	tlbwe	r3,r0,PPC44x_TLB_PAGEID	/* Load the pageid fields */
	tlbwe	r4,r0,PPC44x_TLB_XLAT	/* Load the translation fields */
	tlbwe	r5,r0,PPC44x_TLB_ATTRIB	/* Load the attrib/access fields */

	/* Force context change */
	mfmsr	r0
	mtspr	SPRN_SRR1, r0
	lis	r0,3f@h
	ori	r0,r0,3f@l
	mtspr	SPRN_SRR0,r0
	sync
	rfi

	/* If necessary, invalidate original entry we used */
3:	cmpwi	r23,63
	beq	4f
	li	r6,0
	tlbwe   r6,r23,PPC44x_TLB_PAGEID
	isync

4:
#ifdef CONFIG_PPC_EARLY_DEBUG_44x
	/* Add UART mapping for early debug. */

	/* pageid fields */
	lis	r3,PPC44x_EARLY_DEBUG_VIRTADDR@h
	ori	r3,r3,PPC44x_TLB_VALID|PPC44x_TLB_TS|PPC44x_TLB_64K

	/* xlat fields */
	lis	r4,CONFIG_PPC_EARLY_DEBUG_44x_PHYSLOW@h
	ori	r4,r4,CONFIG_PPC_EARLY_DEBUG_44x_PHYSHIGH

	/* attrib fields */
	li	r5,(PPC44x_TLB_SW|PPC44x_TLB_SR|PPC44x_TLB_I|PPC44x_TLB_G)
        li      r0,62                    /* TLB slot 0 */

	tlbwe	r3,r0,PPC44x_TLB_PAGEID
	tlbwe	r4,r0,PPC44x_TLB_XLAT
	tlbwe	r5,r0,PPC44x_TLB_ATTRIB

	/* Force context change */
	isync
#endif /* CONFIG_PPC_EARLY_DEBUG_44x */

	/* Establish the interrupt vector offsets */
	SET_IVOR(0,  CriticalInput);
	SET_IVOR(1,  MachineCheck);
	SET_IVOR(2,  DataStorage);
	SET_IVOR(3,  InstructionStorage);
	SET_IVOR(4,  ExternalInput);
	SET_IVOR(5,  Alignment);
	SET_IVOR(6,  Program);
	SET_IVOR(7,  FloatingPointUnavailable);
	SET_IVOR(8,  SystemCall);
	SET_IVOR(9,  AuxillaryProcessorUnavailable);
	SET_IVOR(10, Decrementer);
	SET_IVOR(11, FixedIntervalTimer);
	SET_IVOR(12, WatchdogTimer);
	SET_IVOR(13, DataTLBError44x);
	SET_IVOR(14, InstructionTLBError44x);
	SET_IVOR(15, DebugCrit);

	b	head_start_common


#ifdef CONFIG_PPC_47x

#ifdef CONFIG_SMP

/* Entry point for secondary 47x processors */
_GLOBAL(start_secondary_47x)
        mr      r24,r3          /* CPU number */

	bl	init_cpu_state

	/* Now we need to bolt the rest of kernel memory which
	 * is done in C code. We must be careful because our task
	 * struct or our stack can (and will probably) be out
	 * of reach of the initial 256M TLB entry, so we use a
	 * small temporary stack in .bss for that. This works
	 * because only one CPU at a time can be in this code
	 */
	lis	r1,temp_boot_stack@h
	ori	r1,r1,temp_boot_stack@l
	addi	r1,r1,1024-STACK_FRAME_OVERHEAD
	li	r0,0
	stw	r0,0(r1)
	bl	mmu_init_secondary

	/* Now we can get our task struct and real stack pointer */

	/* Get current_thread_info and current */
	lis	r1,secondary_ti@ha
	lwz	r1,secondary_ti@l(r1)
	lwz	r2,TI_TASK(r1)

	/* Current stack pointer */
	addi	r1,r1,THREAD_SIZE-STACK_FRAME_OVERHEAD
	li	r0,0
	stw	r0,0(r1)

	/* Kernel stack for exception entry in SPRG3 */
	addi	r4,r2,THREAD	/* init task's THREAD */
	mtspr	SPRN_SPRG3,r4

	b	start_secondary

#endif /* CONFIG_SMP */


head_start_47x:
	/* Load our current PID->MMUCR TID and MSR IS->MMUCR STS */
	mfspr	r3,SPRN_PID			/* Get PID */
	mfmsr	r4				/* Get MSR */
	andi.	r4,r4,MSR_IS@l			/* TS=1? */
	beq	1f				/* If not, leave STS=0 */
	oris	r3,r3,PPC47x_MMUCR_STS@h	/* Set STS=1 */
1:	mtspr	SPRN_MMUCR,r3			/* Put MMUCR */
	sync

	/* Find the entry we are running from */
	bl	1f
1:	mflr	r23
	tlbsx	r23,0,r23
	tlbre	r24,r23,0
	tlbre	r25,r23,1
	tlbre	r26,r23,2


	/* Initialize MMUCR */
	li	r5,0
	mtspr	SPRN_MMUCR,r5
	sync

clear_all_utlb_entries:

	#; Set initial values.

	addis		r3,0,0x8000
	addi		r4,0,0
	addi		r5,0,0
	b		clear_utlb_entry

	#; Align the loop to speed things up.

	.align		6

clear_utlb_entry:

	tlbwe		r4,r3,0
	tlbwe		r5,r3,1
	tlbwe		r5,r3,2
	addis		r3,r3,0x2000
	cmpwi		r3,0
	bne		clear_utlb_entry
	addis		r3,0,0x8000
	addis		r4,r4,0x100
	cmpwi		r4,0
	bne		clear_utlb_entry

	#; Restore original entry.

	oris	r23,r23,0x8000  /* specify the way */
	tlbwe		r24,r23,0
	tlbwe		r25,r23,1
	tlbwe		r26,r23,2


	lis	r3,PAGE_OFFSET@h
	ori	r3,r3,PAGE_OFFSET@l

	/* Kernel is at the base of RAM */
	li r4, 0			/* Load the kernel physical address */

	/* Load the kernel PID = 0 */
	li	r0,0
	mtspr	SPRN_PID,r0
	sync

	/* Word 0 */
	clrrwi	r3,r3,12		/* Mask off the effective page number */
	ori	r3,r3,PPC47x_TLB0_VALID | PPC47x_TLB0_256M

	/* Word 1 */
	clrrwi	r4,r4,12		/* Mask off the real page number */
					/* ERPN is 0 for first 4GB page */
	/* Word 2 */
	li	r5,0
	ori	r5,r5,PPC47x_TLB2_S_RWX
#ifdef CONFIG_SMP
	ori	r5,r5,PPC47x_TLB2_M
#endif

	/* We write to way 0 and bolted 0 */
	lis	r0,0x8800
	tlbwe	r3,r0,0
	tlbwe	r4,r0,1
	tlbwe	r5,r0,2

	LOAD_REG_IMMEDIATE(r3, 0x9abcdef0)
	mtspr	SPRN_SSPCR,r3
	mtspr	SPRN_USPCR,r3
	LOAD_REG_IMMEDIATE(r3, 0x12345670)
	mtspr	SPRN_ISPCR,r3

	/* Force context change */
	mfmsr	r0
	mtspr	SPRN_SRR1, r0
	lis	r0,3f@h
	ori	r0,r0,3f@l
	mtspr	SPRN_SRR0,r0
	sync
	rfi

	/* Invalidate original entry we used */
3:
	rlwinm	r24,r24,0,21,19 /* clear the "valid" bit */
	tlbwe	r24,r23,0
	addi	r24,0,0
	tlbwe	r24,r23,1
	tlbwe	r24,r23,2
	isync                   /* Clear out the shadow TLB entries */

#ifdef CONFIG_PPC_EARLY_DEBUG_44x
	/* Add UART mapping for early debug. */

	/* Word 0 */
	lis	r3,PPC44x_EARLY_DEBUG_VIRTADDR@h
	ori	r3,r3,PPC47x_TLB0_VALID | PPC47x_TLB0_TS | PPC47x_TLB0_1M

	/* Word 1 */
	lis	r4,CONFIG_PPC_EARLY_DEBUG_44x_PHYSLOW@h
	ori	r4,r4,CONFIG_PPC_EARLY_DEBUG_44x_PHYSHIGH

	/* Word 2 */
	li	r5,(PPC47x_TLB2_S_RW | PPC47x_TLB2_IMG)

	/* Bolted in way 0, bolt slot 5, we -hope- we don't hit the same
	 * congruence class as the kernel, we need to make sure of it at
	 * some point
	 */
        lis	r0,0x8d00
	tlbwe	r3,r0,0
	tlbwe	r4,r0,1
	tlbwe	r5,r0,2

	/* Force context change */
	isync
#endif /* CONFIG_PPC_EARLY_DEBUG_44x */

	/* Establish the interrupt vector offsets */
	SET_IVOR(0,  CriticalInput);
	SET_IVOR(1,  MachineCheckA);
	SET_IVOR(2,  DataStorage);
	SET_IVOR(3,  InstructionStorage);
	SET_IVOR(4,  ExternalInput);
	SET_IVOR(5,  Alignment);
	SET_IVOR(6,  Program);
	SET_IVOR(7,  FloatingPointUnavailable);
	SET_IVOR(8,  SystemCall);
	SET_IVOR(9,  AuxillaryProcessorUnavailable);
	SET_IVOR(10, Decrementer);
	SET_IVOR(11, FixedIntervalTimer);
	SET_IVOR(12, WatchdogTimer);
	SET_IVOR(13, DataTLBError47x);
	SET_IVOR(14, InstructionTLBError47x);
	SET_IVOR(15, DebugCrit);

	/* We configure icbi to invalidate 128 bytes at a time since the
	 * current 32-bit kernel code isn't too happy with icache != dcache
	 * block size
	 */
	mfspr	r3,SPRN_CCR0
	oris	r3,r3,0x0020
	mtspr	SPRN_CCR0,r3
	isync

#endif /* CONFIG_PPC_47x */

head_start_common:
	/* Establish the interrupt vector base */
	lis	r4,interrupt_base@h	/* IVPR only uses the high 16-bits */
	mtspr	SPRN_IVPR,r4

	addis	r22,r22,KERNELBASE@h
	mtlr	r22
	isync
	blr

	.data
	.align	PAGE_SHIFT
	.globl	sdata
sdata:
	.globl	empty_zero_page
empty_zero_page:
	.space	PAGE_SIZE

	.globl	swapper_pg_dir
swapper_pg_dir:
	.space	PGD_TABLE_SIZE

abatron_pteptrs:
	.space	8

#ifdef CONFIG_SMP
	.align	12
temp_boot_stack:
	.space	1024
#endif /* CONFIG_SMP */
