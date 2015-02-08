

#define PSR_BITS_TO_CLEAR						\
	(IA64_PSR_I | IA64_PSR_IT | IA64_PSR_DT | IA64_PSR_RT |		\
	 IA64_PSR_DD | IA64_PSR_SS | IA64_PSR_RI | IA64_PSR_ED |	\
	 IA64_PSR_DFL | IA64_PSR_DFH)

#define PSR_BITS_TO_SET							\
	(IA64_PSR_BN)

#include <asm/processor.h>
#include <asm/asmmacro.h>

GLOBAL_ENTRY(esi_call_phys)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(2)
	alloc loc1=ar.pfs,2,7,8,0
	ld8 r2=[in0],8			// load ESI function's entry point
	mov loc0=rp
	.body
	;;
	ld8 out0=[in1],8		// ESI params loaded from array
	;;				// passing all as inputs doesn't work
	ld8 out1=[in1],8
	;;
	ld8 out2=[in1],8
	;;
	ld8 out3=[in1],8
	;;
	ld8 out4=[in1],8
	;;
	ld8 out5=[in1],8
	;;
	ld8 out6=[in1],8
	;;
	ld8 out7=[in1]
	mov loc2=gp			// save global pointer
	mov loc4=ar.rsc			// save RSE configuration
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	;;
	ld8 gp=[in0]			// load ESI function's global pointer
	movl r16=PSR_BITS_TO_CLEAR
	mov loc3=psr			// save processor status word
	movl r17=PSR_BITS_TO_SET
	;;
	or loc3=loc3,r17
	mov b6=r2
	;;
	andcm r16=loc3,r16	// get psr with IT, DT, and RT bits cleared
	br.call.sptk.many rp=ia64_switch_mode_phys
.ret0:	mov loc5=r19			// old ar.bsp
	mov loc6=r20			// old sp
	br.call.sptk.many rp=b6		// call the ESI function
.ret1:	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	mov r16=loc3			// save virtual mode psr
	mov r19=loc5			// save virtual mode bspstore
	mov r20=loc6			// save virtual mode sp
	br.call.sptk.many rp=ia64_switch_mode_virt // return to virtual mode
.ret2:	mov ar.rsc=loc4			// restore RSE configuration
	mov ar.pfs=loc1
	mov rp=loc0
	mov gp=loc2
	br.ret.sptk.many rp
END(esi_call_phys)
