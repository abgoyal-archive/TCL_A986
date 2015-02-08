

#include <asm/asmmacro.h>

GLOBAL_ENTRY(__strncpy_from_user)
	alloc r2=ar.pfs,3,0,0,0
	mov r8=0
	mov r9=in1
	;;
	add r10=in1,in2
	cmp.eq p6,p0=r0,in2
(p6)	br.ret.spnt.many rp

	// XXX braindead copy loop---this needs to be optimized
.Loop1:
	EX(.Lexit, ld1 r8=[in1],1)
	;;
	EX(.Lexit, st1 [in0]=r8,1)
	cmp.ne p6,p7=r8,r0
	;;
(p6)	cmp.ne.unc p8,p0=in1,r10
(p8)	br.cond.dpnt.few .Loop1
	;;
(p6)	mov r8=in2		// buffer filled up---return buffer length
(p7)	sub r8=in1,r9,1		// return string length (excluding NUL character)
[.Lexit:]
	br.ret.sptk.many rp
END(__strncpy_from_user)
