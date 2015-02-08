

#include "ppc_asm.h"

	.machine "ppc64"

	.text


	.globl __system_reset_overlay
__system_reset_overlay:

	/* Switch to 32-bit mode. */

	mfmsr	r9
	clrldi	r9,r9,1
	mtmsrd	r9
	nop

	/* Get thread number in r3 and branch. */

	mfspr	r3, 0x88
	cntlzw.	r3, r3
	beq	1f

	/* Secondary goes to __secondary_hold in kernel. */

	li	r4, 0x60
	mtctr	r4
	bctr

1:
	/* Save the value at addr zero for a null pointer write check later. */

	li	r4, 0
	lwz	r3, 0(r4)

	/* Primary delays then goes to _zimage_start in wrapper. */

	or	31, 31, 31 /* db16cyc */
	or	31, 31, 31 /* db16cyc */

	lis	r4, _zimage_start@ha
	addi	r4, r4, _zimage_start@l
	mtctr	r4
	bctr

	. = __system_reset_overlay + 512


	.globl __system_reset_kernel
__system_reset_kernel:

	. = __system_reset_kernel + 512
