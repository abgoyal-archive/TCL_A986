
	.file	"div_small.S"


#include "fpu_emu.h"

.text
ENTRY(FPU_div_small)
	pushl	%ebp
	movl	%esp,%ebp

	pushl	%esi

	movl	PARAM1,%esi	/* pointer to num */
	movl	PARAM2,%ecx	/* The denominator */

	movl	4(%esi),%eax	/* Get the current num msw */
	xorl	%edx,%edx
	divl	%ecx

	movl	%eax,4(%esi)

	movl	(%esi),%eax	/* Get the num lsw */
	divl	%ecx

	movl	%eax,(%esi)

	movl	%edx,%eax	/* Return the remainder in eax */

	popl	%esi

	leave
	ret

