



	.text
	.align 4
	.global copy_page
	.ent copy_page
copy_page:
	.prologue 0

	/* Prefetch 5 read cachelines; write-hint 10 cache lines.  */
	wh64	($16)
	ldl	$31,0($17)
	ldl	$31,64($17)
	lda	$1,1*64($16)

	wh64	($1)
	ldl	$31,128($17)
	ldl	$31,192($17)
	lda	$1,2*64($16)

	wh64	($1)
	ldl	$31,256($17)
	lda	$18,118
	lda	$1,3*64($16)

	wh64	($1)
	nop
	lda	$1,4*64($16)
	lda	$2,5*64($16)

	wh64	($1)
	wh64	($2)
	lda	$1,6*64($16)
	lda	$2,7*64($16)

	wh64	($1)
	wh64	($2)
	lda	$1,8*64($16)
	lda	$2,9*64($16)

	wh64	($1)
	wh64	($2)
	lda	$19,10*64($16)
	nop

	/* Main prefetching/write-hinting loop.  */
1:	ldq	$0,0($17)
	ldq	$1,8($17)
	unop
	unop

	unop
	unop
	ldq	$2,16($17)
	ldq	$3,24($17)

	ldq	$4,32($17)
	ldq	$5,40($17)
	unop
	unop

	unop
	unop
	ldq	$6,48($17)
	ldq	$7,56($17)

	ldl	$31,320($17)
	unop
	unop
	unop

	/* This gives the extra cycle of aeration above the minimum.  */
	unop			
	unop
	unop
	unop

	wh64	($19)
	unop
	unop
	unop

	stq	$0,0($16)
	subq	$18,1,$18
	stq	$1,8($16)
	unop

	unop
	stq	$2,16($16)
	addq	$17,64,$17
	stq	$3,24($16)

	stq	$4,32($16)
	stq	$5,40($16)
	addq	$19,64,$19
	unop

	stq	$6,48($16)
	stq	$7,56($16)
	addq	$16,64,$16
	bne	$18, 1b

	/* Prefetch the final 5 cache lines of the read stream.  */
	lda	$18,10
	ldl	$31,320($17)
	ldl	$31,384($17)
	ldl	$31,448($17)

	ldl	$31,512($17)
	ldl	$31,576($17)
	nop
	nop

	/* Non-prefetching, non-write-hinting cleanup loop for the
	   final 10 cache lines.  */
2:	ldq	$0,0($17)
	ldq	$1,8($17)
	ldq	$2,16($17)
	ldq	$3,24($17)

	ldq	$4,32($17)
	ldq	$5,40($17)
	ldq	$6,48($17)
	ldq	$7,56($17)

	stq	$0,0($16)
	subq	$18,1,$18
	stq	$1,8($16)
	addq	$17,64,$17

	stq	$2,16($16)
	stq	$3,24($16)
	stq	$4,32($16)
	stq	$5,40($16)

	stq	$6,48($16)
	stq	$7,56($16)
	addq	$16,64,$16
	bne	$18, 2b

	ret
	nop
	unop
	nop

	.end copy_page
