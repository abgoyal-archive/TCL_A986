

	.set noreorder
	.set noat

	.globl	strlen
	.ent	strlen
	.align 4
strlen:
	ldq_u	$1, 0($16)	# L : load first quadword ($16  may be misaligned)
	lda	$2, -1($31)	# E :
	insqh	$2, $16, $2	# U :
	andnot	$16, 7, $0	# E :

	or	$2, $1, $1	# E :
	cmpbge	$31, $1, $2	# E : $2  <- bitmask: bit i == 1 <==> i-th byte == 0
	nop			# E :
	bne	$2, $found	# U :

$loop:	ldq	$1, 8($0)	# L :
	addq	$0, 8, $0	# E : addr += 8
	cmpbge	$31, $1, $2	# E :
	beq	$2, $loop	# U :

$found:
	cttz	$2, $3		# U0 :
	addq	$0, $3, $0	# E :
	subq	$0, $16, $0	# E :
	ret	$31, ($26)	# L0 :

	.end	strlen
