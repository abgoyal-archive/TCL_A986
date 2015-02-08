


	.text

	.globl	_Atari_get_EdDI_version

/*--- Vector installer ---*/

_Atari_get_EdDI_version:
	movel	sp@(4),a0	/* Value of EdDI cookie */

	/* Call EdDI function #0 */
	clrw	d0
	jsr	(a0)

	rts
