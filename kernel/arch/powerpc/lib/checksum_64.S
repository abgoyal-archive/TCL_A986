

#include <linux/sys.h>
#include <asm/processor.h>
#include <asm/errno.h>
#include <asm/ppc_asm.h>

_GLOBAL(ip_fast_csum)
	lwz	r0,0(r3)
	lwzu	r5,4(r3)
	addic.	r4,r4,-2
	addc	r0,r0,r5
	mtctr	r4
	blelr-
1:	lwzu	r4,4(r3)
	adde	r0,r0,r4
	bdnz	1b
	addze	r0,r0		/* add in final carry */
        rldicl  r4,r0,32,0      /* fold two 32-bit halves together */
        add     r0,r0,r4
        srdi    r0,r0,32
	rlwinm	r3,r0,16,0,31	/* fold two halves together */
	add	r3,r0,r3
	not	r3,r3
	srwi	r3,r3,16
	blr

_GLOBAL(csum_tcpudp_magic)
	rlwimi	r5,r6,16,0,15	/* put proto in upper half of len */
	addc	r0,r3,r4	/* add 4 32-bit words together */
	adde	r0,r0,r5
	adde	r0,r0,r7
        rldicl  r4,r0,32,0      /* fold 64 bit value */
        add     r0,r4,r0
        srdi    r0,r0,32
	rlwinm	r3,r0,16,0,31	/* fold two halves together */
	add	r3,r0,r3
	not	r3,r3
	srwi	r3,r3,16
	blr

_GLOBAL(csum_partial)
        subi	r3,r3,8		/* we'll offset by 8 for the loads */
        srdi.	r6,r4,3         /* divide by 8 for doubleword count */
        addic   r5,r5,0         /* clear carry */
        beq	3f              /* if we're doing < 8 bytes */
        andi.	r0,r3,2         /* aligned on a word boundary already? */
        beq+	1f
        lhz     r6,8(r3)        /* do 2 bytes to get aligned */
        addi    r3,r3,2
        subi    r4,r4,2
        addc    r5,r5,r6
        srdi.   r6,r4,3         /* recompute number of doublewords */
        beq     3f              /* any left? */
1:      mtctr   r6
2:      ldu     r6,8(r3)        /* main sum loop */
        adde    r5,r5,r6
        bdnz    2b
        andi.	r4,r4,7         /* compute bytes left to sum after doublewords */
3:	cmpwi	0,r4,4		/* is at least a full word left? */
	blt	4f
	lwz	r6,8(r3)	/* sum this word */
	addi	r3,r3,4
	subi	r4,r4,4
	adde	r5,r5,r6
4:	cmpwi	0,r4,2		/* is at least a halfword left? */
        blt+	5f
        lhz     r6,8(r3)        /* sum this halfword */
        addi    r3,r3,2
        subi    r4,r4,2
        adde    r5,r5,r6
5:	cmpwi	0,r4,1		/* is at least a byte left? */
        bne+    6f
        lbz     r6,8(r3)        /* sum this byte */
        slwi    r6,r6,8         /* this byte is assumed to be the upper byte of a halfword */
        adde    r5,r5,r6
6:      addze	r5,r5		/* add in final carry */
	rldicl  r4,r5,32,0      /* fold two 32-bit halves together */
        add     r3,r4,r5
        srdi    r3,r3,32
        blr

_GLOBAL(csum_partial_copy_generic)
	addic	r0,r6,0
	subi	r3,r3,4
	subi	r4,r4,4
	srwi.	r6,r5,2
	beq	3f		/* if we're doing < 4 bytes */
	andi.	r9,r4,2		/* Align dst to longword boundary */
	beq+	1f
81:	lhz	r6,4(r3)	/* do 2 bytes to get aligned */
	addi	r3,r3,2
	subi	r5,r5,2
91:	sth	r6,4(r4)
	addi	r4,r4,2
	addc	r0,r0,r6
	srwi.	r6,r5,2		/* # words to do */
	beq	3f
1:	mtctr	r6
82:	lwzu	r6,4(r3)	/* the bdnz has zero overhead, so it should */
92:	stwu	r6,4(r4)	/* be unnecessary to unroll this loop */
	adde	r0,r0,r6
	bdnz	82b
	andi.	r5,r5,3
3:	cmpwi	0,r5,2
	blt+	4f
83:	lhz	r6,4(r3)
	addi	r3,r3,2
	subi	r5,r5,2
93:	sth	r6,4(r4)
	addi	r4,r4,2
	adde	r0,r0,r6
4:	cmpwi	0,r5,1
	bne+	5f
84:	lbz	r6,4(r3)
94:	stb	r6,4(r4)
	slwi	r6,r6,8		/* Upper byte of word */
	adde	r0,r0,r6
5:	addze	r3,r0		/* add in final carry (unlikely with 64-bit regs) */
        rldicl  r4,r3,32,0      /* fold 64 bit value */
        add     r3,r4,r3
        srdi    r3,r3,32
	blr


	.globl src_error_1
src_error_1:
	li	r6,0
	subi	r5,r5,2
95:	sth	r6,4(r4)
	addi	r4,r4,2
	srwi.	r6,r5,2
	beq	3f
	mtctr	r6
	.globl src_error_2
src_error_2:
	li	r6,0
96:	stwu	r6,4(r4)
	bdnz	96b
3:	andi.	r5,r5,3
	beq	src_error
	.globl src_error_3
src_error_3:
	li	r6,0
	mtctr	r5
	addi	r4,r4,3
97:	stbu	r6,1(r4)
	bdnz	97b
	.globl src_error
src_error:
	cmpdi	0,r7,0
	beq	1f
	li	r6,-EFAULT
	stw	r6,0(r7)
1:	addze	r3,r0
	blr

	.globl dst_error
dst_error:
	cmpdi	0,r8,0
	beq	1f
	li	r6,-EFAULT
	stw	r6,0(r8)
1:	addze	r3,r0
	blr

.section __ex_table,"a"
	.align  3
	.llong	81b,src_error_1
	.llong	91b,dst_error
	.llong	82b,src_error_2
	.llong	92b,dst_error
	.llong	83b,src_error_3
	.llong	93b,dst_error
	.llong	84b,src_error_3
	.llong	94b,dst_error
	.llong	95b,dst_error
	.llong	96b,dst_error
	.llong	97b,dst_error
