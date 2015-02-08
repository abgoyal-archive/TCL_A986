
	.file	"reg_u_add.S"


#include "exception.h"
#include "fpu_emu.h"
#include "control_w.h"

.text
ENTRY(FPU_u_add)
	pushl	%ebp
	movl	%esp,%ebp
	pushl	%esi
	pushl	%edi
	pushl	%ebx

	movl	PARAM1,%esi		/* source 1 */
	movl	PARAM2,%edi		/* source 2 */

	movl	PARAM6,%ecx
	movl	%ecx,%edx
	subl	PARAM7,%ecx			/* exp1 - exp2 */
	jge	L_arg1_larger

	/* num1 is smaller */
	movl	SIGL(%esi),%ebx
	movl	SIGH(%esi),%eax

	movl	%edi,%esi
	movl	PARAM7,%edx
	negw	%cx
	jmp	L_accum_loaded

L_arg1_larger:
	/* num1 has larger or equal exponent */
	movl	SIGL(%edi),%ebx
	movl	SIGH(%edi),%eax

L_accum_loaded:
	movl	PARAM3,%edi		/* destination */
	movw	%dx,EXP(%edi)		/* Copy exponent to destination */

	xorl	%edx,%edx		/* clear the extension */

#ifdef PARANOID
	testl	$0x80000000,%eax
	je	L_bugged

	testl	$0x80000000,SIGH(%esi)
	je	L_bugged
#endif /* PARANOID */

/* The number to be shifted is in %eax:%ebx:%edx */
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

	movl	%eax,%edx
	orl	%ebx,%ebx
	jz	L_more_63_no_low

	orl	$1,%edx
	jmp	L_more_63_no_low

L_more_than_64:
	movl	$1,%edx		/* The shifted nr always at least one '1' */

L_more_63_no_low:
	xorl	%ebx,%ebx
	xorl	%eax,%eax

L_shift_done:
	/* Now do the addition */
	addl	SIGL(%esi),%ebx
	adcl	SIGH(%esi),%eax
	jnc	L_round_the_result

	/* Overflow, adjust the result */
	rcrl	$1,%eax
	rcrl	$1,%ebx
	rcrl	$1,%edx
	jnc	L_no_bit_lost

	orl	$1,%edx

L_no_bit_lost:
	incw	EXP(%edi)

L_round_the_result:
	jmp	fpu_reg_round	/* Round the result */



#ifdef PARANOID
/* If we ever get here then we have problems! */
L_bugged:
	pushl	EX_INTERNAL|0x201
	call	EXCEPTION
	pop	%ebx
	movl	$-1,%eax
	jmp	L_exit

L_exit:
	popl	%ebx
	popl	%edi
	popl	%esi
	leave
	ret
#endif /* PARANOID */
