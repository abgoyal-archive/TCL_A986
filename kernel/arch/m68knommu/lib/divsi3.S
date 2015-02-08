



/* These are predefined by new versions of GNU cpp.  */

#ifndef __USER_LABEL_PREFIX__
#define __USER_LABEL_PREFIX__ _
#endif

#ifndef __REGISTER_PREFIX__
#define __REGISTER_PREFIX__
#endif

#ifndef __IMMEDIATE_PREFIX__
#define __IMMEDIATE_PREFIX__ #
#endif

/* ANSI concatenation macros.  */

#define CONCAT1(a, b) CONCAT2(a, b)
#define CONCAT2(a, b) a ## b

/* Use the right prefix for global labels.  */

#define SYM(x) CONCAT1 (__USER_LABEL_PREFIX__, x)

/* Use the right prefix for registers.  */

#define REG(x) CONCAT1 (__REGISTER_PREFIX__, x)

/* Use the right prefix for immediate values.  */

#define IMM(x) CONCAT1 (__IMMEDIATE_PREFIX__, x)

#define d0 REG (d0)
#define d1 REG (d1)
#define d2 REG (d2)
#define d3 REG (d3)
#define d4 REG (d4)
#define d5 REG (d5)
#define d6 REG (d6)
#define d7 REG (d7)
#define a0 REG (a0)
#define a1 REG (a1)
#define a2 REG (a2)
#define a3 REG (a3)
#define a4 REG (a4)
#define a5 REG (a5)
#define a6 REG (a6)
#define fp REG (fp)
#define sp REG (sp)

	.text
	.proc
	.globl	SYM (__divsi3)
SYM (__divsi3):
	movel	d2, sp@-

	moveq	IMM (1), d2	/* sign of result stored in d2 (=1 or =-1) */
	movel	sp@(12), d1	/* d1 = divisor */
	jpl	L1
	negl	d1
#if !(defined(__mcf5200__) || defined(__mcoldfire__))
	negb	d2		/* change sign because divisor <0  */
#else
	negl	d2		/* change sign because divisor <0  */
#endif
L1:	movel	sp@(8), d0	/* d0 = dividend */
	jpl	L2
	negl	d0
#if !(defined(__mcf5200__) || defined(__mcoldfire__))
	negb	d2
#else
	negl	d2
#endif

L2:	movel	d1, sp@-
	movel	d0, sp@-
	jbsr	SYM (__udivsi3)	/* divide abs(dividend) by abs(divisor) */
	addql	IMM (8), sp

	tstb	d2
	jpl	L3
	negl	d0

L3:	movel	sp@+, d2
	rts

