


	.text

	.align 4
	.globl strcat
	.ent strcat
strcat:
	.frame $30, 0, $26
	.prologue 0

	mov	$16, $0		# E : set up return value
	/* Find the end of the string.  */
	ldq_u   $1, 0($16)	# L : load first quadword (a0 may be misaligned)
	lda     $2, -1		# E :
	insqh   $2, $16, $2	# U :

	andnot  $16, 7, $16	# E :
	or      $2, $1, $1	# E :
	cmpbge  $31, $1, $2	# E : bits set iff byte == 0
	bne     $2, $found	# U :

$loop:	ldq     $1, 8($16)	# L :
	addq    $16, 8, $16	# E :
	cmpbge  $31, $1, $2	# E :
	beq     $2, $loop	# U :

$found:	cttz	$2, $3		# U0 :
	addq	$16, $3, $16	# E :
	/* Now do the append.  */
	mov	$26, $23	# E :
	br	__stxcpy	# L0 :

	.end strcat
