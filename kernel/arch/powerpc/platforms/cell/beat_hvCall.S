

#include <asm/ppc_asm.h>

#define	STK_PARM(i)	(48 + ((i)-3)*8)

/* Not implemented on Beat, now */
#define	HCALL_INST_PRECALL
#define	HCALL_INST_POSTCALL

	.text

#define	HVSC	.long	0x44000022

/* Note: takes only 7 input parameters at maximum */
_GLOBAL(beat_hcall_norets)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	mr	r11,r3
	mr	r3,r4
	mr	r4,r5
	mr	r5,r6
	mr	r6,r7
	mr	r7,r8
	mr	r8,r9

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes 8 input parameters at maximum */
_GLOBAL(beat_hcall_norets8)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	mr	r11,r3
	mr	r3,r4
	mr	r4,r5
	mr	r5,r6
	mr	r6,r7
	mr	r7,r8
	mr	r8,r9
	ld	r10,STK_PARM(r10)(r1)

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 1 output parameters at maximum */
_GLOBAL(beat_hcall1)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 2 output parameters at maximum */
_GLOBAL(beat_hcall2)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)
	std	r5,  8(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 3 output parameters at maximum */
_GLOBAL(beat_hcall3)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)
	std	r5,  8(r12)
	std	r6, 16(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 4 output parameters at maximum */
_GLOBAL(beat_hcall4)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)
	std	r5,  8(r12)
	std	r6, 16(r12)
	std	r7, 24(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 5 output parameters at maximum */
_GLOBAL(beat_hcall5)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)
	std	r5,  8(r12)
	std	r6, 16(r12)
	std	r7, 24(r12)
	std	r8, 32(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */

/* Note: takes only 6 input parameters, 6 output parameters at maximum */
_GLOBAL(beat_hcall6)
	HMT_MEDIUM

	mfcr	r0
	stw	r0,8(r1)

	HCALL_INST_PRECALL

	std	r4,STK_PARM(r4)(r1)	/* save ret buffer */

	mr	r11,r3
	mr	r3,r5
	mr	r4,r6
	mr	r5,r7
	mr	r6,r8
	mr	r7,r9
	mr	r8,r10

	HVSC				/* invoke the hypervisor */

	HCALL_INST_POSTCALL

	ld	r12,STK_PARM(r4)(r1)
	std	r4,  0(r12)
	std	r5,  8(r12)
	std	r6, 16(r12)
	std	r7, 24(r12)
	std	r8, 32(r12)
	std	r9, 40(r12)

	lwz	r0,8(r1)
	mtcrf	0xff,r0

	blr				/* return r3 = status */
