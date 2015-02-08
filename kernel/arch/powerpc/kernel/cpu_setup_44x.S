

#include <asm/processor.h>
#include <asm/cputable.h>
#include <asm/ppc_asm.h>

_GLOBAL(__setup_cpu_440ep)
	b	__init_fpu_44x
_GLOBAL(__setup_cpu_440epx)
	mflr	r4
	bl	__init_fpu_44x
	bl	__plb_disable_wrp
	bl	__fixup_440A_mcheck
	mtlr	r4
	blr
_GLOBAL(__setup_cpu_440grx)
	mflr	r4
	bl	__plb_disable_wrp
	bl	__fixup_440A_mcheck
	mtlr	r4
	blr
_GLOBAL(__setup_cpu_460ex)
_GLOBAL(__setup_cpu_460gt)
_GLOBAL(__setup_cpu_460sx)
	mflr	r4
	bl	__init_fpu_44x
	bl	__fixup_440A_mcheck
	mtlr	r4
	blr

_GLOBAL(__setup_cpu_440x5)
_GLOBAL(__setup_cpu_440gx)
_GLOBAL(__setup_cpu_440spe)
	b	__fixup_440A_mcheck

/* enable APU between CPU and FPU */
_GLOBAL(__init_fpu_44x)
	mfspr	r3,SPRN_CCR0
	/* Clear DAPUIB flag in CCR0 */
	rlwinm	r3,r3,0,12,10
	mtspr	SPRN_CCR0,r3
	isync
	blr

#define DCRN_PLB4A0_ACR	0x81

_GLOBAL(__plb_disable_wrp)
	mfdcr	r3,DCRN_PLB4A0_ACR
	/* clear WRP bit in PLB4A0_ACR */
	rlwinm	r3,r3,0,8,6
	mtdcr	DCRN_PLB4A0_ACR,r3
	isync
	blr

