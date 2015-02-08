

	.text

	.align 3
	.globl strcpy
	.ent strcpy
strcpy:
	.frame $30, 0, $26
	.prologue 0

	mov	$16, $0		# set up return value
	mov	$26, $23	# set up return address
	unop
	br	__stxcpy	# do the copy

	.end strcpy
