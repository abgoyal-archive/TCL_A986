

#include <asm/asmmacro.h>
#include <asm/processor.h>

	.data
pal_entry_point:
	data8 ia64_pal_default_handler
	.text

GLOBAL_ENTRY(ia64_pal_handler_init)
	alloc r3=ar.pfs,1,0,0,0
	movl r2=pal_entry_point
	;;
	st8 [r2]=in0
	br.ret.sptk.many rp
END(ia64_pal_handler_init)

GLOBAL_ENTRY(ia64_pal_default_handler)
	mov r8=-1
	br.cond.sptk.many rp
END(ia64_pal_default_handler)

GLOBAL_ENTRY(ia64_pal_call_static)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(4)
	alloc loc1 = ar.pfs,4,5,0,0
	movl loc2 = pal_entry_point
1:	{
	  mov r28 = in0
	  mov r29 = in1
	  mov r8 = ip
	}
	;;
	ld8 loc2 = [loc2]		// loc2 <- entry point
	adds r8 = 1f-1b,r8
	mov loc4=ar.rsc			// save RSE configuration
	;;
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	mov loc3 = psr
	mov loc0 = rp
	.body
	mov r30 = in2

	mov r31 = in3
	mov b7 = loc2

	rsm psr.i
	;;
	mov rp = r8
	br.cond.sptk.many b7
1:	mov psr.l = loc3
	mov ar.rsc = loc4		// restore RSE configuration
	mov ar.pfs = loc1
	mov rp = loc0
	;;
	srlz.d				// seralize restoration of psr.l
	br.ret.sptk.many b0
END(ia64_pal_call_static)

GLOBAL_ENTRY(ia64_pal_call_stacked)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(4)
	alloc loc1 = ar.pfs,4,4,4,0
	movl loc2 = pal_entry_point

	mov r28  = in0			// Index MUST be copied to r28
	mov out0 = in0			// AND in0 of PAL function
	mov loc0 = rp
	.body
	;;
	ld8 loc2 = [loc2]		// loc2 <- entry point
	mov out1 = in1
	mov out2 = in2
	mov out3 = in3
	mov loc3 = psr
	;;
	rsm psr.i
	mov b7 = loc2
	;;
	br.call.sptk.many rp=b7		// now make the call
.ret0:	mov psr.l  = loc3
	mov ar.pfs = loc1
	mov rp = loc0
	;;
	srlz.d				// serialize restoration of psr.l
	br.ret.sptk.many b0
END(ia64_pal_call_stacked)

#define PAL_PSR_BITS_TO_CLEAR						      \
	(IA64_PSR_I | IA64_PSR_IT | IA64_PSR_DT  | IA64_PSR_DB | IA64_PSR_RT |\
	 IA64_PSR_DD | IA64_PSR_SS | IA64_PSR_RI | IA64_PSR_ED |	      \
	 IA64_PSR_DFL | IA64_PSR_DFH)

#define PAL_PSR_BITS_TO_SET						      \
	(IA64_PSR_BN)


GLOBAL_ENTRY(ia64_pal_call_phys_static)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(4)
	alloc loc1 = ar.pfs,4,7,0,0
	movl loc2 = pal_entry_point
1:	{
	  mov r28  = in0		// copy procedure index
	  mov r8   = ip			// save ip to compute branch
	  mov loc0 = rp			// save rp
	}
	.body
	;;
	ld8 loc2 = [loc2]		// loc2 <- entry point
	mov r29  = in1			// first argument
	mov r30  = in2			// copy arg2
	mov r31  = in3			// copy arg3
	;;
	mov loc3 = psr			// save psr
	adds r8  = 1f-1b,r8		// calculate return address for call
	;;
	mov loc4=ar.rsc			// save RSE configuration
	dep.z loc2=loc2,0,61		// convert pal entry point to physical
	tpa r8=r8			// convert rp to physical
	;;
	mov b7 = loc2			// install target to branch reg
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	movl r16=PAL_PSR_BITS_TO_CLEAR
	movl r17=PAL_PSR_BITS_TO_SET
	;;
	or loc3=loc3,r17		// add in psr the bits to set
	;;
	andcm r16=loc3,r16		// removes bits to clear from psr
	br.call.sptk.many rp=ia64_switch_mode_phys
	mov rp = r8			// install return address (physical)
	mov loc5 = r19
	mov loc6 = r20
	br.cond.sptk.many b7
1:
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	mov r16=loc3			// r16= original psr
	mov r19=loc5
	mov r20=loc6
	br.call.sptk.many rp=ia64_switch_mode_virt // return to virtual mode
	mov psr.l = loc3		// restore init PSR

	mov ar.pfs = loc1
	mov rp = loc0
	;;
	mov ar.rsc=loc4			// restore RSE configuration
	srlz.d				// seralize restoration of psr.l
	br.ret.sptk.many b0
END(ia64_pal_call_phys_static)

GLOBAL_ENTRY(ia64_pal_call_phys_stacked)
	.prologue ASM_UNW_PRLG_RP|ASM_UNW_PRLG_PFS, ASM_UNW_PRLG_GRSAVE(5)
	alloc	loc1 = ar.pfs,5,7,4,0
	movl	loc2 = pal_entry_point
1:	{
	  mov r28  = in0		// copy procedure index
	  mov loc0 = rp			// save rp
	}
	.body
	;;
	ld8 loc2 = [loc2]		// loc2 <- entry point
	mov loc3 = psr			// save psr
	;;
	mov loc4=ar.rsc			// save RSE configuration
	dep.z loc2=loc2,0,61		// convert pal entry point to physical
	;;
	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	movl r16=PAL_PSR_BITS_TO_CLEAR
	movl r17=PAL_PSR_BITS_TO_SET
	;;
	or loc3=loc3,r17		// add in psr the bits to set
	mov b7 = loc2			// install target to branch reg
	;;
	andcm r16=loc3,r16		// removes bits to clear from psr
	br.call.sptk.many rp=ia64_switch_mode_phys

	mov out0 = in0			// first argument
	mov out1 = in1			// copy arg2
	mov out2 = in2			// copy arg3
	mov out3 = in3			// copy arg3
	mov loc5 = r19
	mov loc6 = r20

	br.call.sptk.many rp=b7		// now make the call

	mov ar.rsc=0			// put RSE in enforced lazy, LE mode
	mov r16=loc3			// r16= original psr
	mov r19=loc5
	mov r20=loc6
	br.call.sptk.many rp=ia64_switch_mode_virt // return to virtual mode

	mov psr.l  = loc3		// restore init PSR
	mov ar.pfs = loc1
	mov rp = loc0
	;;
	mov ar.rsc=loc4			// restore RSE configuration
	srlz.d				// seralize restoration of psr.l
	br.ret.sptk.many b0
END(ia64_pal_call_phys_stacked)

GLOBAL_ENTRY(ia64_save_scratch_fpregs)
	alloc r3=ar.pfs,1,0,0,0
	add r2=16,in0
	;;
	stf.spill [in0] = f10,32
	stf.spill [r2]  = f11,32
	;;
	stf.spill [in0] = f12,32
	stf.spill [r2]  = f13,32
	;;
	stf.spill [in0] = f14,32
	stf.spill [r2]  = f15,32
	br.ret.sptk.many rp
END(ia64_save_scratch_fpregs)

GLOBAL_ENTRY(ia64_load_scratch_fpregs)
	alloc r3=ar.pfs,1,0,0,0
	add r2=16,in0
	;;
	ldf.fill  f10 = [in0],32
	ldf.fill  f11 = [r2],32
	;;
	ldf.fill  f12 = [in0],32
	ldf.fill  f13 = [r2],32
	;;
	ldf.fill  f14 = [in0],32
	ldf.fill  f15 = [r2],32
	br.ret.sptk.many rp
END(ia64_load_scratch_fpregs)
