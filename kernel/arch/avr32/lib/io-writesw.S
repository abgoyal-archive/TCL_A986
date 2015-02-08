

.Lnot_word_aligned:
	ld.uh	r8, r11++
	sub	r10, 1
	st.h	r12[0], r8

	.global	__raw_writesw
	.type	__raw_writesw,@function
__raw_writesw:
	cp.w	r10, 0
	mov	r9, 3
	reteq	r12
	tst	r11, r9
	brne	.Lnot_word_aligned

	sub	r10, 2
	brlt	2f

1:	ld.w	r8, r11++
	bfextu	r9, r8, 16, 16
	st.h	r12[0], r9
	st.h	r12[0], r8
	sub	r10, 2
	brge	1b

2:	sub	r10, -2
	reteq	r12

	ld.uh	r8, r11++
	st.h	r12[0], r8
	retal	r12
