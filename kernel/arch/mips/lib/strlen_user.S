
#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>

#define EX(insn,reg,addr,handler)			\
9:	insn	reg, addr;				\
	.section __ex_table,"a";			\
	PTR	9b, handler;				\
	.previous

LEAF(__strlen_user_asm)
	LONG_L		v0, TI_ADDR_LIMIT($28)	# pointer ok?
	and		v0, a0
	bnez		v0, .Lfault

FEXPORT(__strlen_user_nocheck_asm)
	move		v0, a0
1:	EX(lb, t0, (v0), .Lfault)
	PTR_ADDIU	v0, 1
	bnez		t0, 1b
	PTR_SUBU	v0, a0
	jr		ra
	END(__strlen_user_asm)

.Lfault:	move		v0, zero
	jr		ra
