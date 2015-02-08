
#ifndef _ASM_POWERPC_EXCEPTION_H
#define _ASM_POWERPC_EXCEPTION_H

#define EX_R9		0
#define EX_R10		8
#define EX_R11		16
#define EX_R12		24
#define EX_R13		32
#define EX_SRR0		40
#define EX_DAR		48
#define EX_DSISR	56
#define EX_CCR		60
#define EX_R3		64
#define EX_LR		72

#define LOAD_HANDLER(reg, label)					\
	addi	reg,reg,(label)-_stext;	/* virt addr of handler ... */

#define EXCEPTION_PROLOG_1(area)				\
	mfspr	r13,SPRN_SPRG_PACA;	/* get paca address into r13 */	\
	std	r9,area+EX_R9(r13);	/* save r9 - r12 */		\
	std	r10,area+EX_R10(r13);					\
	std	r11,area+EX_R11(r13);					\
	std	r12,area+EX_R12(r13);					\
	mfspr	r9,SPRN_SPRG_SCRATCH0;					\
	std	r9,area+EX_R13(r13);					\
	mfcr	r9

#define EXCEPTION_PROLOG_PSERIES_1(label)				\
	ld	r12,PACAKBASE(r13);	/* get high part of &label */	\
	ld	r10,PACAKMSR(r13);	/* get MSR value for kernel */	\
	mfspr	r11,SPRN_SRR0;		/* save SRR0 */			\
	LOAD_HANDLER(r12,label)						\
	mtspr	SPRN_SRR0,r12;						\
	mfspr	r12,SPRN_SRR1;		/* and SRR1 */			\
	mtspr	SPRN_SRR1,r10;						\
	rfid;								\
	b	.	/* prevent speculative execution */

#define EXCEPTION_PROLOG_PSERIES(area, label)				\
	EXCEPTION_PROLOG_1(area);					\
	EXCEPTION_PROLOG_PSERIES_1(label);

#define EXCEPTION_PROLOG_COMMON(n, area)				   \
	andi.	r10,r12,MSR_PR;		/* See if coming from user	*/ \
	mr	r10,r1;			/* Save r1			*/ \
	subi	r1,r1,INT_FRAME_SIZE;	/* alloc frame on kernel stack	*/ \
	beq-	1f;							   \
	ld	r1,PACAKSAVE(r13);	/* kernel stack to use		*/ \
1:	cmpdi	cr1,r1,0;		/* check if r1 is in userspace	*/ \
	bge-	cr1,2f;			/* abort if it is		*/ \
	b	3f;							   \
2:	li	r1,(n);			/* will be reloaded later	*/ \
	sth	r1,PACA_TRAP_SAVE(r13);					   \
	b	bad_stack;						   \
3:	std	r9,_CCR(r1);		/* save CR in stackframe	*/ \
	std	r11,_NIP(r1);		/* save SRR0 in stackframe	*/ \
	std	r12,_MSR(r1);		/* save SRR1 in stackframe	*/ \
	std	r10,0(r1);		/* make stack chain pointer	*/ \
	std	r0,GPR0(r1);		/* save r0 in stackframe	*/ \
	std	r10,GPR1(r1);		/* save r1 in stackframe	*/ \
	ACCOUNT_CPU_USER_ENTRY(r9, r10);				   \
	std	r2,GPR2(r1);		/* save r2 in stackframe	*/ \
	SAVE_4GPRS(3, r1);		/* save r3 - r6 in stackframe	*/ \
	SAVE_2GPRS(7, r1);		/* save r7, r8 in stackframe	*/ \
	ld	r9,area+EX_R9(r13);	/* move r9, r10 to stackframe	*/ \
	ld	r10,area+EX_R10(r13);					   \
	std	r9,GPR9(r1);						   \
	std	r10,GPR10(r1);						   \
	ld	r9,area+EX_R11(r13);	/* move r11 - r13 to stackframe	*/ \
	ld	r10,area+EX_R12(r13);					   \
	ld	r11,area+EX_R13(r13);					   \
	std	r9,GPR11(r1);						   \
	std	r10,GPR12(r1);						   \
	std	r11,GPR13(r1);						   \
	ld	r2,PACATOC(r13);	/* get kernel TOC into r2	*/ \
	mflr	r9;			/* save LR in stackframe	*/ \
	std	r9,_LINK(r1);						   \
	mfctr	r10;			/* save CTR in stackframe	*/ \
	std	r10,_CTR(r1);						   \
	lbz	r10,PACASOFTIRQEN(r13);				   \
	mfspr	r11,SPRN_XER;		/* save XER in stackframe	*/ \
	std	r10,SOFTE(r1);						   \
	std	r11,_XER(r1);						   \
	li	r9,(n)+1;						   \
	std	r9,_TRAP(r1);		/* set trap number		*/ \
	li	r10,0;							   \
	ld	r11,exception_marker@toc(r2);				   \
	std	r10,RESULT(r1);		/* clear regs->result		*/ \
	std	r11,STACK_FRAME_OVERHEAD-16(r1); /* mark the frame	*/

#define STD_EXCEPTION_PSERIES(n, label)			\
	. = n;						\
	.globl label##_pSeries;				\
label##_pSeries:					\
	HMT_MEDIUM;					\
	DO_KVM	n;					\
	mtspr	SPRN_SPRG_SCRATCH0,r13;		/* save r13 */	\
	EXCEPTION_PROLOG_PSERIES(PACA_EXGEN, label##_common)

#define HSTD_EXCEPTION_PSERIES(n, label)		\
	. = n;						\
	.globl label##_pSeries;				\
label##_pSeries:					\
	HMT_MEDIUM;					\
	mtspr	SPRN_SPRG_SCRATCH0,r20;	/* save r20 */	\
	mfspr	r20,SPRN_HSRR0;		/* copy HSRR0 to SRR0 */ \
	mtspr	SPRN_SRR0,r20;				\
	mfspr	r20,SPRN_HSRR1;		/* copy HSRR0 to SRR0 */ \
	mtspr	SPRN_SRR1,r20;				\
	mfspr	r20,SPRN_SPRG_SCRATCH0;	/* restore r20 */ \
	mtspr	SPRN_SPRG_SCRATCH0,r13;		/* save r13 */	\
	EXCEPTION_PROLOG_PSERIES(PACA_EXGEN, label##_common)


#define MASKABLE_EXCEPTION_PSERIES(n, label)				\
	. = n;								\
	.globl label##_pSeries;						\
label##_pSeries:							\
	HMT_MEDIUM;							\
	DO_KVM	n;							\
	mtspr	SPRN_SPRG_SCRATCH0,r13;	/* save r13 */			\
	mfspr	r13,SPRN_SPRG_PACA;	/* get paca address into r13 */	\
	std	r9,PACA_EXGEN+EX_R9(r13);	/* save r9, r10 */	\
	std	r10,PACA_EXGEN+EX_R10(r13);				\
	lbz	r10,PACASOFTIRQEN(r13);					\
	mfcr	r9;							\
	cmpwi	r10,0;							\
	beq	masked_interrupt;					\
	mfspr	r10,SPRN_SPRG_SCRATCH0;					\
	std	r10,PACA_EXGEN+EX_R13(r13);				\
	std	r11,PACA_EXGEN+EX_R11(r13);				\
	std	r12,PACA_EXGEN+EX_R12(r13);				\
	ld	r12,PACAKBASE(r13);	/* get high part of &label */	\
	ld	r10,PACAKMSR(r13);	/* get MSR value for kernel */	\
	mfspr	r11,SPRN_SRR0;		/* save SRR0 */			\
	LOAD_HANDLER(r12,label##_common)				\
	mtspr	SPRN_SRR0,r12;						\
	mfspr	r12,SPRN_SRR1;		/* and SRR1 */			\
	mtspr	SPRN_SRR1,r10;						\
	rfid;								\
	b	.	/* prevent speculative execution */

#ifdef CONFIG_PPC_ISERIES
#define DISABLE_INTS				\
	li	r11,0;				\
	stb	r11,PACASOFTIRQEN(r13);		\
BEGIN_FW_FTR_SECTION;				\
	stb	r11,PACAHARDIRQEN(r13);		\
END_FW_FTR_SECTION_IFCLR(FW_FEATURE_ISERIES);	\
	TRACE_DISABLE_INTS;			\
BEGIN_FW_FTR_SECTION;				\
	mfmsr	r10;				\
	ori	r10,r10,MSR_EE;			\
	mtmsrd	r10,1;				\
END_FW_FTR_SECTION_IFSET(FW_FEATURE_ISERIES)
#else
#define DISABLE_INTS				\
	li	r11,0;				\
	stb	r11,PACASOFTIRQEN(r13);		\
	stb	r11,PACAHARDIRQEN(r13);		\
	TRACE_DISABLE_INTS
#endif /* CONFIG_PPC_ISERIES */

#define ENABLE_INTS				\
	ld	r12,_MSR(r1);			\
	mfmsr	r11;				\
	rlwimi	r11,r12,0,MSR_EE;		\
	mtmsrd	r11,1

#define STD_EXCEPTION_COMMON(trap, label, hdlr)		\
	.align	7;					\
	.globl label##_common;				\
label##_common:						\
	EXCEPTION_PROLOG_COMMON(trap, PACA_EXGEN);	\
	DISABLE_INTS;					\
	bl	.save_nvgprs;				\
	addi	r3,r1,STACK_FRAME_OVERHEAD;		\
	bl	hdlr;					\
	b	.ret_from_except

#define STD_EXCEPTION_COMMON_IDLE(trap, label, hdlr)	\
	.align	7;					\
	.globl label##_common;				\
label##_common:						\
	EXCEPTION_PROLOG_COMMON(trap, PACA_EXGEN);	\
	FINISH_NAP;					\
	DISABLE_INTS;					\
	bl	.save_nvgprs;				\
	addi	r3,r1,STACK_FRAME_OVERHEAD;		\
	bl	hdlr;					\
	b	.ret_from_except

#define STD_EXCEPTION_COMMON_LITE(trap, label, hdlr)	\
	.align	7;					\
	.globl label##_common;				\
label##_common:						\
	EXCEPTION_PROLOG_COMMON(trap, PACA_EXGEN);	\
	FINISH_NAP;					\
	DISABLE_INTS;					\
BEGIN_FTR_SECTION					\
	bl	.ppc64_runlatch_on;			\
END_FTR_SECTION_IFSET(CPU_FTR_CTRL)			\
	addi	r3,r1,STACK_FRAME_OVERHEAD;		\
	bl	hdlr;					\
	b	.ret_from_except_lite

#ifdef CONFIG_PPC_970_NAP
#define FINISH_NAP				\
BEGIN_FTR_SECTION				\
	clrrdi	r11,r1,THREAD_SHIFT;		\
	ld	r9,TI_LOCAL_FLAGS(r11);		\
	andi.	r10,r9,_TLF_NAPPING;		\
	bnel	power4_fixup_nap;		\
END_FTR_SECTION_IFSET(CPU_FTR_CAN_NAP)
#else
#define FINISH_NAP
#endif

#endif	/* _ASM_POWERPC_EXCEPTION_H */
