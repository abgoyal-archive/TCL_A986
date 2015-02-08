

#define PSR_BITS_TO_CLEAR						\
	(IA64_PSR_I | IA64_PSR_IT | IA64_PSR_DT | IA64_PSR_RT |		\
	 IA64_PSR_DD | IA64_PSR_SS | IA64_PSR_RI | IA64_PSR_ED |	\
	 IA64_PSR_DFL | IA64_PSR_DFH)

#define PSR_BITS_TO_SET							\
	(IA64_PSR_BN)

#include <asm/processor.h>
#include <asm/asmmacro.h>


GLOBAL_ENTRY(efi_call_phys)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(8)
	alloc loc1=ar.pfs,8,7,7,0
	ld8 r2=[in0],8			// load EFI function's entry point
	mov loc0=rp
	.body
	;;
	mov loc2=gp			// save global pointer
	mov loc4=ar.rsc			// save RSE configuration
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	;;
	ld8 gp=[in0]			// load EFI function's global pointer
	movl r16=PSR_BITS_TO_CLEAR
	mov loc3=psr			// save processor status word
	movl r17=PSR_BITS_TO_SET
	;;
	or loc3=loc3,r17
	mov b6=r2
	;;
	andcm r16=loc3,r16	// get psr with IT, DT, and RT bits cleared
	br.call.sptk.many rp=ia64_switch_mode_phys
.ret0:	mov out4=in5
	mov out0=in1
	mov out1=in2
	mov out2=in3
	mov out3=in4
	mov out5=in6
	mov out6=in7
	mov loc5=r19
	mov loc6=r20
	br.call.sptk.many rp=b6		// call the EFI function
.ret1:	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	mov r16=loc3
	mov r19=loc5
	mov r20=loc6
	br.call.sptk.many rp=ia64_switch_mode_virt // return to virtual mode
.ret2:	mov ar.rsc=loc4			// restore RSE configuration
	mov ar.pfs=loc1
	mov rp=loc0
	mov gp=loc2
	br.ret.sptk.many rp
END(efi_call_phys)
