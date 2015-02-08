

#include <linux/init.h>
#include <asm/processor.h>
#include <asm/page.h>
#include <asm/mmu.h>
#include <asm/pgtable.h>
#include <asm/cputable.h>
#include <asm/thread_info.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>

	__HEAD
_ENTRY(_stext);
_ENTRY(_start);

	/* Save parameters we are passed.
	*/
	mr	r31,r3
	mr	r30,r4
	mr	r29,r5
	mr	r28,r6
	mr	r27,r7

	/* We have to turn on the MMU right away so we get cache modes
	 * set correctly.
	 */
	bl	initial_mmu

turn_on_mmu:
	lis	r0,MSR_KERNEL@h
	ori	r0,r0,MSR_KERNEL@l
	mtspr	SPRN_SRR1,r0
	lis	r0,start_here@h
	ori	r0,r0,start_here@l
	mtspr	SPRN_SRR0,r0
	SYNC
	rfi				/* enables MMU */
	b	.			/* prevent prefetch past rfi */

	. = 0xc0
crit_save:
_ENTRY(crit_r10)
	.space	4
_ENTRY(crit_r11)
	.space	4
_ENTRY(crit_srr0)
	.space	4
_ENTRY(crit_srr1)
	.space	4
_ENTRY(saved_ksp_limit)
	.space	4

#define NORMAL_EXCEPTION_PROLOG						     \
	mtspr	SPRN_SPRG_SCRATCH0,r10;	/* save two registers to work with */\
	mtspr	SPRN_SPRG_SCRATCH1,r11;					     \
	mtspr	SPRN_SPRG_SCRATCH2,r1;					     \
	mfcr	r10;			/* save CR in r10 for now	   */\
	mfspr	r11,SPRN_SRR1;		/* check whether user or kernel    */\
	andi.	r11,r11,MSR_PR;						     \
	beq	1f;							     \
	mfspr	r1,SPRN_SPRG_THREAD;	/* if from user, start at top of   */\
	lwz	r1,THREAD_INFO-THREAD(r1); /* this thread's kernel stack   */\
	addi	r1,r1,THREAD_SIZE;					     \
1:	subi	r1,r1,INT_FRAME_SIZE;	/* Allocate an exception frame     */\
	tophys(r11,r1);							     \
	stw	r10,_CCR(r11);          /* save various registers	   */\
	stw	r12,GPR12(r11);						     \
	stw	r9,GPR9(r11);						     \
	mfspr	r10,SPRN_SPRG_SCRATCH0;					     \
	stw	r10,GPR10(r11);						     \
	mfspr	r12,SPRN_SPRG_SCRATCH1;					     \
	stw	r12,GPR11(r11);						     \
	mflr	r10;							     \
	stw	r10,_LINK(r11);						     \
	mfspr	r10,SPRN_SPRG_SCRATCH2;					     \
	mfspr	r12,SPRN_SRR0;						     \
	stw	r10,GPR1(r11);						     \
	mfspr	r9,SPRN_SRR1;						     \
	stw	r10,0(r11);						     \
	rlwinm	r9,r9,0,14,12;		/* clear MSR_WE (necessary?)	   */\
	stw	r0,GPR0(r11);						     \
	SAVE_4GPRS(3, r11);						     \
	SAVE_2GPRS(7, r11)

#define CRITICAL_EXCEPTION_PROLOG					     \
	stw	r10,crit_r10@l(0);	/* save two registers to work with */\
	stw	r11,crit_r11@l(0);					     \
	mfcr	r10;			/* save CR in r10 for now	   */\
	mfspr	r11,SPRN_SRR3;		/* check whether user or kernel    */\
	andi.	r11,r11,MSR_PR;						     \
	lis	r11,critirq_ctx@ha;					     \
	tophys(r11,r11);						     \
	lwz	r11,critirq_ctx@l(r11);					     \
	beq	1f;							     \
	/* COMING FROM USER MODE */					     \
	mfspr	r11,SPRN_SPRG_THREAD;	/* if from user, start at top of   */\
	lwz	r11,THREAD_INFO-THREAD(r11); /* this thread's kernel stack */\
1:	addi	r11,r11,THREAD_SIZE-INT_FRAME_SIZE; /* Alloc an excpt frm  */\
	tophys(r11,r11);						     \
	stw	r10,_CCR(r11);          /* save various registers	   */\
	stw	r12,GPR12(r11);						     \
	stw	r9,GPR9(r11);						     \
	mflr	r10;							     \
	stw	r10,_LINK(r11);						     \
	mfspr	r12,SPRN_DEAR;		/* save DEAR and ESR in the frame  */\
	stw	r12,_DEAR(r11);		/* since they may have had stuff   */\
	mfspr	r9,SPRN_ESR;		/* in them at the point where the  */\
	stw	r9,_ESR(r11);		/* exception was taken		   */\
	mfspr	r12,SPRN_SRR2;						     \
	stw	r1,GPR1(r11);						     \
	mfspr	r9,SPRN_SRR3;						     \
	stw	r1,0(r11);						     \
	tovirt(r1,r11);							     \
	rlwinm	r9,r9,0,14,12;		/* clear MSR_WE (necessary?)	   */\
	stw	r0,GPR0(r11);						     \
	SAVE_4GPRS(3, r11);						     \
	SAVE_2GPRS(7, r11)

	/*
	 * State at this point:
	 * r9 saved in stack frame, now saved SRR3 & ~MSR_WE
	 * r10 saved in crit_r10 and in stack frame, trashed
	 * r11 saved in crit_r11 and in stack frame,
	 *	now phys stack/exception frame pointer
	 * r12 saved in stack frame, now saved SRR2
	 * CR saved in stack frame, CR0.EQ = !SRR3.PR
	 * LR, DEAR, ESR in stack frame
	 * r1 saved in stack frame, now virt stack/excframe pointer
	 * r0, r3-r8 saved in stack frame
	 */

#define	START_EXCEPTION(n, label)					     \
	. = n;								     \
label:

#define EXCEPTION(n, label, hdlr, xfer)				\
	START_EXCEPTION(n, label);				\
	NORMAL_EXCEPTION_PROLOG;				\
	addi	r3,r1,STACK_FRAME_OVERHEAD;			\
	xfer(n, hdlr)

#define CRITICAL_EXCEPTION(n, label, hdlr)			\
	START_EXCEPTION(n, label);				\
	CRITICAL_EXCEPTION_PROLOG;				\
	addi	r3,r1,STACK_FRAME_OVERHEAD;			\
	EXC_XFER_TEMPLATE(hdlr, n+2, (MSR_KERNEL & ~(MSR_ME|MSR_DE|MSR_CE)), \
			  NOCOPY, crit_transfer_to_handler,	\
			  ret_from_crit_exc)

#define EXC_XFER_TEMPLATE(hdlr, trap, msr, copyee, tfer, ret)	\
	li	r10,trap;					\
	stw	r10,_TRAP(r11);					\
	lis	r10,msr@h;					\
	ori	r10,r10,msr@l;					\
	copyee(r10, r9);					\
	bl	tfer;		 				\
	.long	hdlr;						\
	.long	ret

#define COPY_EE(d, s)		rlwimi d,s,0,16,16
#define NOCOPY(d, s)

#define EXC_XFER_STD(n, hdlr)		\
	EXC_XFER_TEMPLATE(hdlr, n, MSR_KERNEL, NOCOPY, transfer_to_handler_full, \
			  ret_from_except_full)

#define EXC_XFER_LITE(n, hdlr)		\
	EXC_XFER_TEMPLATE(hdlr, n+1, MSR_KERNEL, NOCOPY, transfer_to_handler, \
			  ret_from_except)

#define EXC_XFER_EE(n, hdlr)		\
	EXC_XFER_TEMPLATE(hdlr, n, MSR_KERNEL, COPY_EE, transfer_to_handler_full, \
			  ret_from_except_full)

#define EXC_XFER_EE_LITE(n, hdlr)	\
	EXC_XFER_TEMPLATE(hdlr, n+1, MSR_KERNEL, COPY_EE, transfer_to_handler, \
			  ret_from_except)


	CRITICAL_EXCEPTION(0x0100, CriticalInterrupt, unknown_exception)

	CRITICAL_EXCEPTION(0x0200, MachineCheck, machine_check_exception)

	START_EXCEPTION(0x0300,	DataStorage)
	mtspr	SPRN_SPRG_SCRATCH0, r10 /* Save some working registers */
	mtspr	SPRN_SPRG_SCRATCH1, r11
#ifdef CONFIG_403GCX
	stw     r12, 0(r0)
	stw     r9, 4(r0)
	mfcr    r11
	mfspr   r12, SPRN_PID
	stw     r11, 8(r0)
	stw     r12, 12(r0)
#else
	mtspr	SPRN_SPRG_SCRATCH3, r12
	mtspr	SPRN_SPRG_SCRATCH4, r9
	mfcr	r11
	mfspr	r12, SPRN_PID
	mtspr	SPRN_SPRG_SCRATCH6, r11
	mtspr	SPRN_SPRG_SCRATCH5, r12
#endif

	/* First, check if it was a zone fault (which means a user
	* tried to access a kernel or read-protected page - always
	* a SEGV).  All other faults here must be stores, so no
	* need to check ESR_DST as well. */
	mfspr	r10, SPRN_ESR
	andis.	r10, r10, ESR_DIZ@h
	bne	2f

	mfspr	r10, SPRN_DEAR		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11, PAGE_OFFSET@h
	cmplw	r10, r11
	blt+	3f
	lis	r11, swapper_pg_dir@h
	ori	r11, r11, swapper_pg_dir@l
	li	r9, 0
	mtspr	SPRN_PID, r9		/* TLB will have 0 TID */
	b	4f

	/* Get the PGD for the current thread.
	 */
3:
	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)
4:
	tophys(r11, r11)
	rlwimi	r11, r10, 12, 20, 29	/* Create L1 (pgdir/pmd) address */
	lwz	r11, 0(r11)		/* Get L1 entry */
	rlwinm.	r12, r11, 0, 0, 19	/* Extract L2 (pte) base address */
	beq	2f			/* Bail if no table */

	rlwimi	r12, r10, 22, 20, 29	/* Compute PTE address */
	lwz	r11, 0(r12)		/* Get Linux PTE */

	andi.	r9, r11, _PAGE_RW	/* Is it writeable? */
	beq	2f			/* Bail if not */

	/* Update 'changed'.
	*/
	ori	r11, r11, _PAGE_DIRTY|_PAGE_ACCESSED|_PAGE_HWWRITE
	stw	r11, 0(r12)		/* Update Linux page table */

	/* Most of the Linux PTE is ready to load into the TLB LO.
	 * We set ZSEL, where only the LS-bit determines user access.
	 * We set execute, because we don't have the granularity to
	 * properly set this at the page level (Linux problem).
	 * If shared is set, we cause a zero PID->TID load.
	 * Many of these bits are software only.  Bits we don't set
	 * here we (properly should) assume have the appropriate value.
	 */
	li	r12, 0x0ce2
	andc	r11, r11, r12		/* Make sure 20, 21 are zero */

	/* find the TLB index that caused the fault.  It has to be here.
	*/
	tlbsx	r9, 0, r10

	tlbwe	r11, r9, TLB_DATA		/* Load TLB LO */

	/* Done...restore registers and get out of here.
	*/
#ifdef CONFIG_403GCX
	lwz     r12, 12(r0)
	lwz     r11, 8(r0)
	mtspr   SPRN_PID, r12
	mtcr    r11
	lwz     r9, 4(r0)
	lwz     r12, 0(r0)
#else
	mfspr	r12, SPRN_SPRG_SCRATCH5
	mfspr	r11, SPRN_SPRG_SCRATCH6
	mtspr	SPRN_PID, r12
	mtcr	r11
	mfspr	r9, SPRN_SPRG_SCRATCH4
	mfspr	r12, SPRN_SPRG_SCRATCH3
#endif
	mfspr	r11, SPRN_SPRG_SCRATCH1
	mfspr	r10, SPRN_SPRG_SCRATCH0
	PPC405_ERR77_SYNC
	rfi			/* Should sync shadow TLBs */
	b	.		/* prevent prefetch past rfi */

2:
	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
#ifdef CONFIG_403GCX
	lwz     r12, 12(r0)
	lwz     r11, 8(r0)
	mtspr   SPRN_PID, r12
	mtcr    r11
	lwz     r9, 4(r0)
	lwz     r12, 0(r0)
#else
	mfspr	r12, SPRN_SPRG_SCRATCH5
	mfspr	r11, SPRN_SPRG_SCRATCH6
	mtspr	SPRN_PID, r12
	mtcr	r11
	mfspr	r9, SPRN_SPRG_SCRATCH4
	mfspr	r12, SPRN_SPRG_SCRATCH3
#endif
	mfspr	r11, SPRN_SPRG_SCRATCH1
	mfspr	r10, SPRN_SPRG_SCRATCH0
	b	DataAccess

	START_EXCEPTION(0x0400, InstructionAccess)
	NORMAL_EXCEPTION_PROLOG
	mr	r4,r12			/* Pass SRR0 as arg2 */
	li	r5,0			/* Pass zero as arg3 */
	EXC_XFER_EE_LITE(0x400, handle_page_fault)

/* 0x0500 - External Interrupt Exception */
	EXCEPTION(0x0500, HardwareInterrupt, do_IRQ, EXC_XFER_LITE)

/* 0x0600 - Alignment Exception */
	START_EXCEPTION(0x0600, Alignment)
	NORMAL_EXCEPTION_PROLOG
	mfspr	r4,SPRN_DEAR		/* Grab the DEAR and save it */
	stw	r4,_DEAR(r11)
	addi	r3,r1,STACK_FRAME_OVERHEAD
	EXC_XFER_EE(0x600, alignment_exception)

/* 0x0700 - Program Exception */
	START_EXCEPTION(0x0700, ProgramCheck)
	NORMAL_EXCEPTION_PROLOG
	mfspr	r4,SPRN_ESR		/* Grab the ESR and save it */
	stw	r4,_ESR(r11)
	addi	r3,r1,STACK_FRAME_OVERHEAD
	EXC_XFER_STD(0x700, program_check_exception)

	EXCEPTION(0x0800, Trap_08, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x0900, Trap_09, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x0A00, Trap_0A, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x0B00, Trap_0B, unknown_exception, EXC_XFER_EE)

/* 0x0C00 - System Call Exception */
	START_EXCEPTION(0x0C00,	SystemCall)
	NORMAL_EXCEPTION_PROLOG
	EXC_XFER_EE_LITE(0xc00, DoSyscall)

	EXCEPTION(0x0D00, Trap_0D, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x0E00, Trap_0E, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x0F00, Trap_0F, unknown_exception, EXC_XFER_EE)

/* 0x1000 - Programmable Interval Timer (PIT) Exception */
	START_EXCEPTION(0x1000, Decrementer)
	NORMAL_EXCEPTION_PROLOG
	lis	r0,TSR_PIS@h
	mtspr	SPRN_TSR,r0		/* Clear the PIT exception */
	addi	r3,r1,STACK_FRAME_OVERHEAD
	EXC_XFER_LITE(0x1000, timer_interrupt)

#if 0

	STND_EXCEPTION(0x1010,	FITException,		unknown_exception)

#ifdef CONFIG_BOOKE_WDT
	CRITICAL_EXCEPTION(0x1020, WDTException, WatchdogException)
#else
	CRITICAL_EXCEPTION(0x1020, WDTException, unknown_exception)
#endif
#endif

	START_EXCEPTION(0x1100,	DTLBMiss)
	mtspr	SPRN_SPRG_SCRATCH0, r10 /* Save some working registers */
	mtspr	SPRN_SPRG_SCRATCH1, r11
#ifdef CONFIG_403GCX
	stw     r12, 0(r0)
	stw     r9, 4(r0)
	mfcr    r11
	mfspr   r12, SPRN_PID
	stw     r11, 8(r0)
	stw     r12, 12(r0)
#else
	mtspr	SPRN_SPRG_SCRATCH3, r12
	mtspr	SPRN_SPRG_SCRATCH4, r9
	mfcr	r11
	mfspr	r12, SPRN_PID
	mtspr	SPRN_SPRG_SCRATCH6, r11
	mtspr	SPRN_SPRG_SCRATCH5, r12
#endif
	mfspr	r10, SPRN_DEAR		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11, PAGE_OFFSET@h
	cmplw	r10, r11
	blt+	3f
	lis	r11, swapper_pg_dir@h
	ori	r11, r11, swapper_pg_dir@l
	li	r9, 0
	mtspr	SPRN_PID, r9		/* TLB will have 0 TID */
	b	4f

	/* Get the PGD for the current thread.
	 */
3:
	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)
4:
	tophys(r11, r11)
	rlwimi	r11, r10, 12, 20, 29	/* Create L1 (pgdir/pmd) address */
	lwz	r12, 0(r11)		/* Get L1 entry */
	andi.	r9, r12, _PMD_PRESENT	/* Check if it points to a PTE page */
	beq	2f			/* Bail if no table */

	rlwimi	r12, r10, 22, 20, 29	/* Compute PTE address */
	lwz	r11, 0(r12)		/* Get Linux PTE */
	andi.	r9, r11, _PAGE_PRESENT
	beq	5f

	ori	r11, r11, _PAGE_ACCESSED
	stw	r11, 0(r12)

	/* Create TLB tag.  This is the faulting address plus a static
	 * set of bits.  These are size, valid, E, U0.
	*/
	li	r12, 0x00c0
	rlwimi	r10, r12, 0, 20, 31

	b	finish_tlb_load

2:	/* Check for possible large-page pmd entry */
	rlwinm.	r9, r12, 2, 22, 24
	beq	5f

	/* Create TLB tag.  This is the faulting address, plus a static
	 * set of bits (valid, E, U0) plus the size from the PMD.
	 */
	ori	r9, r9, 0x40
	rlwimi	r10, r9, 0, 20, 31
	mr	r11, r12

	b	finish_tlb_load

5:
	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
#ifdef CONFIG_403GCX
	lwz     r12, 12(r0)
	lwz     r11, 8(r0)
	mtspr   SPRN_PID, r12
	mtcr    r11
	lwz     r9, 4(r0)
	lwz     r12, 0(r0)
#else
	mfspr	r12, SPRN_SPRG_SCRATCH5
	mfspr	r11, SPRN_SPRG_SCRATCH6
	mtspr	SPRN_PID, r12
	mtcr	r11
	mfspr	r9, SPRN_SPRG_SCRATCH4
	mfspr	r12, SPRN_SPRG_SCRATCH3
#endif
	mfspr	r11, SPRN_SPRG_SCRATCH1
	mfspr	r10, SPRN_SPRG_SCRATCH0
	b	DataAccess

	START_EXCEPTION(0x1200,	ITLBMiss)
	mtspr	SPRN_SPRG_SCRATCH0, r10	 /* Save some working registers */
	mtspr	SPRN_SPRG_SCRATCH1, r11
#ifdef CONFIG_403GCX
	stw     r12, 0(r0)
	stw     r9, 4(r0)
	mfcr    r11
	mfspr   r12, SPRN_PID
	stw     r11, 8(r0)
	stw     r12, 12(r0)
#else
	mtspr	SPRN_SPRG_SCRATCH3, r12
	mtspr	SPRN_SPRG_SCRATCH4, r9
	mfcr	r11
	mfspr	r12, SPRN_PID
	mtspr	SPRN_SPRG_SCRATCH6, r11
	mtspr	SPRN_SPRG_SCRATCH5, r12
#endif
	mfspr	r10, SPRN_SRR0		/* Get faulting address */

	/* If we are faulting a kernel address, we have to use the
	 * kernel page tables.
	 */
	lis	r11, PAGE_OFFSET@h
	cmplw	r10, r11
	blt+	3f
	lis	r11, swapper_pg_dir@h
	ori	r11, r11, swapper_pg_dir@l
	li	r9, 0
	mtspr	SPRN_PID, r9		/* TLB will have 0 TID */
	b	4f

	/* Get the PGD for the current thread.
	 */
3:
	mfspr	r11,SPRN_SPRG_THREAD
	lwz	r11,PGDIR(r11)
4:
	tophys(r11, r11)
	rlwimi	r11, r10, 12, 20, 29	/* Create L1 (pgdir/pmd) address */
	lwz	r12, 0(r11)		/* Get L1 entry */
	andi.	r9, r12, _PMD_PRESENT	/* Check if it points to a PTE page */
	beq	2f			/* Bail if no table */

	rlwimi	r12, r10, 22, 20, 29	/* Compute PTE address */
	lwz	r11, 0(r12)		/* Get Linux PTE */
	andi.	r9, r11, _PAGE_PRESENT
	beq	5f

	ori	r11, r11, _PAGE_ACCESSED
	stw	r11, 0(r12)

	/* Create TLB tag.  This is the faulting address plus a static
	 * set of bits.  These are size, valid, E, U0.
	*/
	li	r12, 0x00c0
	rlwimi	r10, r12, 0, 20, 31

	b	finish_tlb_load

2:	/* Check for possible large-page pmd entry */
	rlwinm.	r9, r12, 2, 22, 24
	beq	5f

	/* Create TLB tag.  This is the faulting address, plus a static
	 * set of bits (valid, E, U0) plus the size from the PMD.
	 */
	ori	r9, r9, 0x40
	rlwimi	r10, r9, 0, 20, 31
	mr	r11, r12

	b	finish_tlb_load

5:
	/* The bailout.  Restore registers to pre-exception conditions
	 * and call the heavyweights to help us out.
	 */
#ifdef CONFIG_403GCX
	lwz     r12, 12(r0)
	lwz     r11, 8(r0)
	mtspr   SPRN_PID, r12
	mtcr    r11
	lwz     r9, 4(r0)
	lwz     r12, 0(r0)
#else
	mfspr	r12, SPRN_SPRG_SCRATCH5
	mfspr	r11, SPRN_SPRG_SCRATCH6
	mtspr	SPRN_PID, r12
	mtcr	r11
	mfspr	r9, SPRN_SPRG_SCRATCH4
	mfspr	r12, SPRN_SPRG_SCRATCH3
#endif
	mfspr	r11, SPRN_SPRG_SCRATCH1
	mfspr	r10, SPRN_SPRG_SCRATCH0
	b	InstructionAccess

	EXCEPTION(0x1300, Trap_13, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1400, Trap_14, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1500, Trap_15, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1600, Trap_16, unknown_exception, EXC_XFER_EE)
#ifdef CONFIG_IBM405_ERR51
	/* 405GP errata 51 */
	START_EXCEPTION(0x1700, Trap_17)
	b DTLBMiss
#else
	EXCEPTION(0x1700, Trap_17, unknown_exception, EXC_XFER_EE)
#endif
	EXCEPTION(0x1800, Trap_18, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1900, Trap_19, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1A00, Trap_1A, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1B00, Trap_1B, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1C00, Trap_1C, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1D00, Trap_1D, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1E00, Trap_1E, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x1F00, Trap_1F, unknown_exception, EXC_XFER_EE)

	/* 0x2000 - Debug Exception */
	START_EXCEPTION(0x2000, DebugTrap)
	CRITICAL_EXCEPTION_PROLOG

	/*
	 * If this is a single step or branch-taken exception in an
	 * exception entry sequence, it was probably meant to apply to
	 * the code where the exception occurred (since exception entry
	 * doesn't turn off DE automatically).  We simulate the effect
	 * of turning off DE on entry to an exception handler by turning
	 * off DE in the SRR3 value and clearing the debug status.
	 */
	mfspr	r10,SPRN_DBSR		/* check single-step/branch taken */
	andis.	r10,r10,DBSR_IC@h
	beq+	2f

	andi.	r10,r9,MSR_IR|MSR_PR	/* check supervisor + MMU off */
	beq	1f			/* branch and fix it up */

	mfspr   r10,SPRN_SRR2		/* Faulting instruction address */
	cmplwi  r10,0x2100
	bgt+    2f			/* address above exception vectors */

	/* here it looks like we got an inappropriate debug exception. */
1:	rlwinm	r9,r9,0,~MSR_DE		/* clear DE in the SRR3 value */
	lis	r10,DBSR_IC@h		/* clear the IC event */
	mtspr	SPRN_DBSR,r10
	/* restore state and get out */
	lwz	r10,_CCR(r11)
	lwz	r0,GPR0(r11)
	lwz	r1,GPR1(r11)
	mtcrf	0x80,r10
	mtspr	SPRN_SRR2,r12
	mtspr	SPRN_SRR3,r9
	lwz	r9,GPR9(r11)
	lwz	r12,GPR12(r11)
	lwz	r10,crit_r10@l(0)
	lwz	r11,crit_r11@l(0)
	PPC405_ERR77_SYNC
	rfci
	b	.

	/* continue normal handling for a critical exception... */
2:	mfspr	r4,SPRN_DBSR
	addi	r3,r1,STACK_FRAME_OVERHEAD
	EXC_XFER_TEMPLATE(DebugException, 0x2002, \
		(MSR_KERNEL & ~(MSR_ME|MSR_DE|MSR_CE)), \
		NOCOPY, crit_transfer_to_handler, ret_from_crit_exc)

DataAccess:
	NORMAL_EXCEPTION_PROLOG
	mfspr	r5,SPRN_ESR		/* Grab the ESR, save it, pass arg3 */
	stw	r5,_ESR(r11)
	mfspr	r4,SPRN_DEAR		/* Grab the DEAR, save it, pass arg2 */
	EXC_XFER_EE_LITE(0x300, handle_page_fault)


	/* Damn, I came up one instruction too many to fit into the
	 * exception space :-).  Both the instruction and data TLB
	 * miss get to this point to load the TLB.
	 * 	r10 - TLB_TAG value
	 * 	r11 - Linux PTE
	 *	r12, r9 - avilable to use
	 *	PID - loaded with proper value when we get here
	 *	Upon exit, we reload everything and RFI.
	 * Actually, it will fit now, but oh well.....a common place
	 * to load the TLB.
	 */
tlb_4xx_index:
	.long	0
finish_tlb_load:
	/* load the next available TLB index.
	*/
	lwz	r9, tlb_4xx_index@l(0)
	addi	r9, r9, 1
	andi.	r9, r9, (PPC40X_TLB_SIZE-1)
	stw	r9, tlb_4xx_index@l(0)

6:
	/*
	 * Clear out the software-only bits in the PTE to generate the
	 * TLB_DATA value.  These are the bottom 2 bits of the RPM, the
	 * top 3 bits of the zone field, and M.
	 */
	li	r12, 0x0ce2
	andc	r11, r11, r12

	tlbwe	r11, r9, TLB_DATA		/* Load TLB LO */
	tlbwe	r10, r9, TLB_TAG		/* Load TLB HI */

	/* Done...restore registers and get out of here.
	*/
#ifdef CONFIG_403GCX
	lwz     r12, 12(r0)
	lwz     r11, 8(r0)
	mtspr   SPRN_PID, r12
	mtcr    r11
	lwz     r9, 4(r0)
	lwz     r12, 0(r0)
#else
	mfspr	r12, SPRN_SPRG_SCRATCH5
	mfspr	r11, SPRN_SPRG_SCRATCH6
	mtspr	SPRN_PID, r12
	mtcr	r11
	mfspr	r9, SPRN_SPRG_SCRATCH4
	mfspr	r12, SPRN_SPRG_SCRATCH3
#endif
	mfspr	r11, SPRN_SPRG_SCRATCH1
	mfspr	r10, SPRN_SPRG_SCRATCH0
	PPC405_ERR77_SYNC
	rfi			/* Should sync shadow TLBs */
	b	.		/* prevent prefetch past rfi */

_ENTRY(giveup_fpu)
	blr

start_here:

	/* ptr to current */
	lis	r2,init_task@h
	ori	r2,r2,init_task@l

	/* ptr to phys current thread */
	tophys(r4,r2)
	addi	r4,r4,THREAD	/* init task's THREAD */
	mtspr	SPRN_SPRG_THREAD,r4

	/* stack */
	lis	r1,init_thread_union@ha
	addi	r1,r1,init_thread_union@l
	li	r0,0
	stwu	r0,THREAD_SIZE-STACK_FRAME_OVERHEAD(r1)

	bl	early_init	/* We have to do this with MMU on */

	mr	r3,r31
	mr	r4,r30
	mr	r5,r29
	mr	r6,r28
	mr	r7,r27
	bl	machine_init
	bl	MMU_init

	lis	r4,2f@h
	ori	r4,r4,2f@l
	tophys(r4,r4)
	lis	r3,(MSR_KERNEL & ~(MSR_IR|MSR_DR))@h
	ori	r3,r3,(MSR_KERNEL & ~(MSR_IR|MSR_DR))@l
	mtspr	SPRN_SRR0,r4
	mtspr	SPRN_SRR1,r3
	rfi
	b	.		/* prevent prefetch past rfi */

/* Load up the kernel context */
2:
	sync			/* Flush to memory before changing TLB */
	tlbia
	isync			/* Flush shadow TLBs */

	/* set up the PTE pointers for the Abatron bdiGDB.
	*/
	lis	r6, swapper_pg_dir@h
	ori	r6, r6, swapper_pg_dir@l
	lis	r5, abatron_pteptrs@h
	ori	r5, r5, abatron_pteptrs@l
	stw	r5, 0xf0(r0)	/* Must match your Abatron config file */
	tophys(r5,r5)
	stw	r6, 0(r5)

/* Now turn on the MMU for real! */
	lis	r4,MSR_KERNEL@h
	ori	r4,r4,MSR_KERNEL@l
	lis	r3,start_kernel@h
	ori	r3,r3,start_kernel@l
	mtspr	SPRN_SRR0,r3
	mtspr	SPRN_SRR1,r4
	rfi			/* enable MMU and jump to start_kernel */
	b	.		/* prevent prefetch past rfi */

initial_mmu:
	tlbia			/* Invalidate all TLB entries */
	isync

	/* We should still be executing code at physical address 0x0000xxxx
	 * at this point. However, start_here is at virtual address
	 * 0xC000xxxx. So, set up a TLB mapping to cover this once
	 * translation is enabled.
	 */

	lis	r3,KERNELBASE@h		/* Load the kernel virtual address */
	ori	r3,r3,KERNELBASE@l
	tophys(r4,r3)			/* Load the kernel physical address */

	iccci	r0,r3			/* Invalidate the i-cache before use */

	/* Load the kernel PID.
	*/
	li	r0,0
	mtspr	SPRN_PID,r0
	sync

	/* Configure and load two entries into TLB slots 62 and 63.
	 * In case we are pinning TLBs, these are reserved in by the
	 * other TLB functions.  If not reserving, then it doesn't
	 * matter where they are loaded.
	 */
	clrrwi	r4,r4,10		/* Mask off the real page number */
	ori	r4,r4,(TLB_WR | TLB_EX)	/* Set the write and execute bits */

	clrrwi	r3,r3,10		/* Mask off the effective page number */
	ori	r3,r3,(TLB_VALID | TLB_PAGESZ(PAGESZ_16M))

        li      r0,63                    /* TLB slot 63 */

	tlbwe	r4,r0,TLB_DATA		/* Load the data portion of the entry */
	tlbwe	r3,r0,TLB_TAG		/* Load the tag portion of the entry */

#if defined(CONFIG_SERIAL_TEXT_DEBUG) && defined(SERIAL_DEBUG_IO_BASE)

	/* Load a TLB entry for the UART, so that ppc4xx_progress() can use
	 * the UARTs nice and early.  We use a 4k real==virtual mapping. */

	lis	r3,SERIAL_DEBUG_IO_BASE@h
	ori	r3,r3,SERIAL_DEBUG_IO_BASE@l
	mr	r4,r3
	clrrwi	r4,r4,12
	ori	r4,r4,(TLB_WR|TLB_I|TLB_M|TLB_G)

	clrrwi	r3,r3,12
	ori	r3,r3,(TLB_VALID | TLB_PAGESZ(PAGESZ_4K))

	li	r0,0			/* TLB slot 0 */
	tlbwe	r4,r0,TLB_DATA
	tlbwe	r3,r0,TLB_TAG
#endif /* CONFIG_SERIAL_DEBUG_TEXT && SERIAL_DEBUG_IO_BASE */

	isync

	/* Establish the exception vector base
	*/
	lis	r4,KERNELBASE@h		/* EVPR only uses the high 16-bits */
	tophys(r0,r4)			/* Use the physical address */
	mtspr	SPRN_EVPR,r0

	blr

_GLOBAL(abort)
        mfspr   r13,SPRN_DBCR0
        oris    r13,r13,DBCR0_RST_SYSTEM@h
        mtspr   SPRN_DBCR0,r13

_GLOBAL(set_context)

#ifdef CONFIG_BDI_SWITCH
	/* Context switch the PTE pointer for the Abatron BDI2000.
	 * The PGDIR is the second parameter.
	 */
	lis	r5, KERNELBASE@h
	lwz	r5, 0xf0(r5)
	stw	r4, 0x4(r5)
#endif
	sync
	mtspr	SPRN_PID,r3
	isync				/* Need an isync to flush shadow */
					/* TLBs after changing PID */
	blr

	.data
	.align	12
	.globl	sdata
sdata:
	.globl	empty_zero_page
empty_zero_page:
	.space	4096
	.globl	swapper_pg_dir
swapper_pg_dir:
	.space	PGD_TABLE_SIZE

abatron_pteptrs:
	.space	8
