

#include <asm/reg.h>
#include <asm/ppc_asm.h>

	.text

_GLOBAL(as1_readb)
	mfmsr	r7
	ori	r0,r7,MSR_DS
	sync
	mtmsr	r0
	sync
	isync
	lbz	r3,0(r3)
	sync
	mtmsr	r7
	sync
	isync
	blr

_GLOBAL(as1_writeb)
	mfmsr	r7
	ori	r0,r7,MSR_DS
	sync
	mtmsr	r0
	sync
	isync
	stb	r3,0(r4)
	sync
	mtmsr	r7
	sync
	isync
	blr
