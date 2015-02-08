

	/*
	 * DWtype __avr32_lsl64(DWtype u, word_type b)
	 */
	.text
	.global	__avr32_lsl64
	.type	__avr32_lsl64,@function
__avr32_lsl64:
	cp.w	r12, 0
	reteq	r12

	rsub	r9, r12, 32
	brle	1f

	lsr	r8, r10, r9
	lsl	r10, r10, r12
	lsl	r11, r11, r12
	or	r11, r8
	retal	r12

1:	neg	r9
	lsl	r11, r10, r9
	mov	r10, 0
	retal	r12
