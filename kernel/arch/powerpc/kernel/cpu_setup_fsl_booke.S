

#include <asm/processor.h>
#include <asm/cputable.h>
#include <asm/ppc_asm.h>

_GLOBAL(__e500_icache_setup)
	mfspr	r0, SPRN_L1CSR1
	andi.	r3, r0, L1CSR1_ICE
	bnelr				/* Already enabled */
	oris	r0, r0, L1CSR1_CPE@h
	ori	r0, r0, (L1CSR1_ICFI | L1CSR1_ICLFR |  L1CSR1_ICE)
	mtspr	SPRN_L1CSR1, r0		/* Enable I-Cache */
	isync
	blr

_GLOBAL(__e500_dcache_setup)
	mfspr	r0, SPRN_L1CSR0
	andi.	r3, r0, L1CSR0_DCE
	bnelr				/* Already enabled */
	msync
	isync
	li	r0, 0
	mtspr	SPRN_L1CSR0, r0		/* Disable */
	msync
	isync
	li	r0, (L1CSR0_DCFI | L1CSR0_CLFC)
	mtspr	SPRN_L1CSR0, r0		/* Invalidate */
	isync
1:	mfspr	r0, SPRN_L1CSR0
	andi.	r3, r0, L1CSR0_CLFC
	bne+	1b			/* Wait for lock bits reset */
	oris	r0, r0, L1CSR0_CPE@h
	ori	r0, r0, L1CSR0_DCE
	msync
	isync
	mtspr	SPRN_L1CSR0, r0		/* Enable */
	isync
	blr

_GLOBAL(__setup_cpu_e200)
	/* enable dedicated debug exception handling resources (Debug APU) */
	mfspr	r3,SPRN_HID0
	ori	r3,r3,HID0_DAPUEN@l
	mtspr	SPRN_HID0,r3
	b	__setup_e200_ivors
_GLOBAL(__setup_cpu_e500v1)
_GLOBAL(__setup_cpu_e500v2)
	mflr	r4
	bl	__e500_icache_setup
	bl	__e500_dcache_setup
	bl	__setup_e500_ivors
	mtlr	r4
	blr
_GLOBAL(__setup_cpu_e500mc)
	mflr	r4
	bl	__e500_icache_setup
	bl	__e500_dcache_setup
	bl	__setup_e500mc_ivors
	mtlr	r4
	blr
