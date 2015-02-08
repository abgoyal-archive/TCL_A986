
#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>

#define EX(insn,reg,addr,handler)			\
9:	insn	reg, addr;				\
	.section __ex_table,"a";			\
	PTR	9b, handler;				\
	.previous

LEAF(__strnlen_user_asm)
	LONG_L		v0, TI_ADDR_LIMIT($28)	# pointer ok?
	and		v0, a0
	bnez		v0, .Lfault

FEXPORT(__strnlen_user_nocheck_asm)
	move		v0, a0
	PTR_ADDU	a1, a0			# stop pointer
1:	beq		v0, a1, 1f		# limit reached?
	EX(lb, t0, (v0), .Lfault)
	PTR_ADDU	v0, 1
	bnez		t0, 1b
1:	PTR_SUBU	v0, a0
	jr		ra
	END(__strnlen_user_asm)

.Lfault:
	move		v0, zero
	jr		ra
