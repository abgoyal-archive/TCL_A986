

	.global	__raw_readsl
	.type	__raw_readsl,@function
__raw_readsl:
	cp.w	r10, 0
	reteq	r12

	/*
	 * If r11 isn't properly aligned, we might get an exception on
	 * some implementations. But there's not much we can do about it.
	 */
1:	ld.w	r8, r12[0]
	sub	r10, 1
	st.w	r11++, r8
	brne	1b

	retal	r12
