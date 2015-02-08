

#include <asm/ppc_asm.h>
#include <asm/processor.h>
#include <asm/ptrace.h>		/* XXX for STACK_FRAME_OVERHEAD */

	.text


_GLOBAL(HvCall)
_GLOBAL(HvCall0)
_GLOBAL(HvCall1)
_GLOBAL(HvCall2)
_GLOBAL(HvCall3)
_GLOBAL(HvCall4)
_GLOBAL(HvCall5)
_GLOBAL(HvCall6)
_GLOBAL(HvCall7)


	mfcr	r0
	std	r0,-8(r1)
	stdu	r1,-(STACK_FRAME_OVERHEAD+16)(r1)

	/* r0 = 0xffffffffffffffff indicates a hypervisor call */

	li	r0,-1

	/* Invoke the hypervisor */

	sc

	ld	r1,0(r1)
	ld	r0,-8(r1)
	mtcrf	0xff,r0

	/*  return to caller, return value in r3 */

	blr

_GLOBAL(HvCall0Ret16)
_GLOBAL(HvCall1Ret16)
_GLOBAL(HvCall2Ret16)
_GLOBAL(HvCall3Ret16)
_GLOBAL(HvCall4Ret16)
_GLOBAL(HvCall5Ret16)
_GLOBAL(HvCall6Ret16)
_GLOBAL(HvCall7Ret16)

	mfcr	r0
	std	r0,-8(r1)
	std	r31,-16(r1)
	stdu	r1,-(STACK_FRAME_OVERHEAD+32)(r1)

	mr	r31,r4
	li	r0,-1
	mr	r4,r5
	mr	r5,r6
	mr	r6,r7
	mr	r7,r8
	mr	r8,r9
	mr	r9,r10

	sc

	std	r3,0(r31)
	std	r4,8(r31)

	mr	r3,r5

	ld	r1,0(r1)
	ld	r0,-8(r1)
	mtcrf	0xff,r0
	ld	r31,-16(r1)

	blr
