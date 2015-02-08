

#include <asm/asmmacro.h>

GLOBAL_ENTRY(__strnlen_user)
	.prologue
	alloc r2=ar.pfs,2,0,0,0
	.save ar.lc, r16
	mov r16=ar.lc			// preserve ar.lc

	.body

	add r3=-1,in1
	;;
	mov ar.lc=r3
	mov r9=0
	;;
	// XXX braindead strlen loop---this needs to be optimized
.Loop1:
	EXCLR(.Lexit, ld1 r8=[in0],1)
	add r9=1,r9
	;;
	cmp.eq p6,p0=r8,r0
(p6)	br.cond.dpnt .Lexit
	br.cloop.dptk.few .Loop1

	add r9=1,in1			// NUL not found---return N+1
	;;
.Lexit:
	mov r8=r9
	mov ar.lc=r16			// restore ar.lc
	br.ret.sptk.many rp
END(__strnlen_user)
