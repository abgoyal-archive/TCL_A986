
	.file	"reg_u_sub.S"


#include "exception.h"
#include "fpu_emu.h"
#include "control_w.h"

.text
ENTRY(FPU_u_sub)
	pushl	%ebp
	movl	%esp,%ebp
	pushl	%esi
	pushl	%edi
	pushl	%ebx

	movl	PARAM1,%esi	/* source 1 */
	movl	PARAM2,%edi	/* source 2 */
	
	movl	PARAM6,%ecx
	subl	PARAM7,%ecx	/* exp1 - exp2 */

#ifdef PARANOID
	/* source 2 is always smaller than source 1 */
	js	L_bugged_1

	testl	$0x80000000,SIGH(%edi)	/* The args are assumed to be be normalized */
	je	L_bugged_2

	testl	$0x80000000,SIGH(%esi)
	je	L_bugged_2
#endif /* PARANOID */

	movl	SIGH(%edi),%eax	/* register ms word */
	movl	SIGL(%edi),%ebx	/* register ls word */

	movl	PARAM3,%edi	/* destination */
	movl	PARAM6,%edx
	movw	%dx,EXP(%edi)	/* Copy exponent to destination */

	xorl	%edx,%edx	/* register extension */


	cmpw	$32,%cx		/* shrd only works for 0..31 bits */
	jnc	L_more_than_31

/* less than 32 bits */
	shrd	%cl,%ebx,%edx
	shrd	%cl,%eax,%ebx
	shr	%cl,%eax
	jmp	L_shift_done

L_more_than_31:
	cmpw	$64,%cx
	jnc	L_more_than_63

	subb	$32,%cl
	jz	L_exactly_32

	shrd	%cl,%eax,%edx
	shr	%cl,%eax
	orl	%ebx,%ebx
	jz	L_more_31_no_low	/* none of the lowest bits is set */

	orl	$1,%edx			/* record the fact in the extension */

L_more_31_no_low:
	movl	%eax,%ebx
	xorl	%eax,%eax
	jmp	L_shift_done

L_exactly_32:
	movl	%ebx,%edx
	movl	%eax,%ebx
	xorl	%eax,%eax
	jmp	L_shift_done

L_more_than_63:
	cmpw	$65,%cx
	jnc	L_more_than_64

	/* Shift right by 64 bits */
	movl	%eax,%edx
	orl	%ebx,%ebx
	jz	L_more_63_no_low

	orl	$1,%edx
	jmp	L_more_63_no_low

L_more_than_64:
	jne	L_more_than_65

	/* Shift right by 65 bits */
	/* Carry is clear if we get here */
	movl	%eax,%edx
	rcrl	%edx
	jnc	L_shift_65_nc

	orl	$1,%edx
	jmp	L_more_63_no_low

L_shift_65_nc:
	orl	%ebx,%ebx
	jz	L_more_63_no_low

	orl	$1,%edx
	jmp	L_more_63_no_low

L_more_than_65:
	movl	$1,%edx		/* The shifted nr always at least one '1' */

L_more_63_no_low:
	xorl	%ebx,%ebx
	xorl	%eax,%eax

L_shift_done:
L_subtr:
	xorl	%ecx,%ecx
	subl	%edx,%ecx
	movl	%ecx,%edx
	movl	SIGL(%esi),%ecx
	sbbl	%ebx,%ecx
	movl	%ecx,%ebx
	movl	SIGH(%esi),%ecx
	sbbl	%eax,%ecx
	movl	%ecx,%eax

#ifdef PARANOID
	/* We can never get a borrow */
	jc	L_bugged
#endif /* PARANOID */

	testl	$0x80000000,%eax
	jnz	L_round		/* no shifting needed */

	orl	%eax,%eax
	jnz	L_shift_1	/* shift left 1 - 31 bits */

	orl	%ebx,%ebx
	jnz	L_shift_32	/* shift left 32 - 63 bits */


	cmpl	$0x80000000,%edx
	jnz	L_must_be_zero

	/* Shift left 64 bits */
	subw	$64,EXP(%edi)
	xchg	%edx,%eax
	jmp	fpu_reg_round

L_must_be_zero:
#ifdef PARANOID
	orl	%edx,%edx
	jnz	L_bugged_3
#endif /* PARANOID */ 

	/* The result is zero */
	movw	$0,EXP(%edi)		/* exponent */
	movl	$0,SIGL(%edi)
	movl	$0,SIGH(%edi)
	movl	TAG_Zero,%eax
	jmp	L_exit

L_shift_32:
	movl	%ebx,%eax
	movl	%edx,%ebx
	movl	$0,%edx
	subw	$32,EXP(%edi)	/* Can get underflow here */

/* We need to shift left by 1 - 31 bits */
L_shift_1:
	bsrl	%eax,%ecx	/* get the required shift in %ecx */
	subl	$31,%ecx
	negl	%ecx
	shld	%cl,%ebx,%eax
	shld	%cl,%edx,%ebx
	shl	%cl,%edx
	subw	%cx,EXP(%edi)	/* Can get underflow here */

L_round:
	jmp	fpu_reg_round	/* Round the result */


#ifdef PARANOID
L_bugged_1:
	pushl	EX_INTERNAL|0x206
	call	EXCEPTION
	pop	%ebx
	jmp	L_error_exit

L_bugged_2:
	pushl	EX_INTERNAL|0x209
	call	EXCEPTION
	pop	%ebx
	jmp	L_error_exit

L_bugged_3:
	pushl	EX_INTERNAL|0x210
	call	EXCEPTION
	pop	%ebx
	jmp	L_error_exit

L_bugged_4:
	pushl	EX_INTERNAL|0x211
	call	EXCEPTION
	pop	%ebx
	jmp	L_error_exit

L_bugged:
	pushl	EX_INTERNAL|0x212
	call	EXCEPTION
	pop	%ebx
	jmp	L_error_exit

L_error_exit:
	movl	$-1,%eax

#endif /* PARANOID */

L_exit:
	popl	%ebx
	popl	%edi
	popl	%esi
	leave
	ret
