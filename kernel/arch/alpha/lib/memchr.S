


        .set noreorder
        .set noat

	.globl memchr
	.ent memchr
memchr:
	.frame $30,0,$26,0
	.prologue 0

	# Hack -- if someone passes in (size_t)-1, hoping to just
	# search til the end of the address space, we will overflow
	# below when we find the address of the last byte.  Given
	# that we will never have a 56-bit address space, cropping
	# the length is the easiest way to avoid trouble.
	zap	$18, 0x80, $5	#-e0	:

	beq	$18, $not_found	# .. e1 :
        ldq_u   $1, 0($16)	# e1	: load first quadword
	insbl	$17, 1, $2	# .. e0 : $2 = 000000000000ch00
	and	$17, 0xff, $17	#-e0    : $17 = 00000000000000ch
	cmpult	$18, 9, $4	# .. e1 :
	or	$2, $17, $17	# e0    : $17 = 000000000000chch
        lda     $3, -1($31)	# .. e1 :
	sll	$17, 16, $2	#-e0    : $2 = 00000000chch0000
	addq	$16, $5, $5	# .. e1 :
	or	$2, $17, $17	# e1    : $17 = 00000000chchchch
	unop			#	:
	sll	$17, 32, $2	#-e0    : $2 = chchchch00000000
	or	$2, $17, $17	# e1	: $17 = chchchchchchchch
	extql	$1, $16, $7	# e0    : 
	beq	$4, $first_quad	# .. e1 :

	ldq_u	$6, -1($5)	#-e1	: eight or less bytes to search
	extqh	$6, $16, $6	# .. e0 :
	mov	$16, $0		# e0	:
	or	$7, $6, $1	# .. e1 : $1 = quadword starting at $16

	# Deal with the case where at most 8 bytes remain to be searched
	# in $1.  E.g.:
	#	$18 = 6
	#	$1 = ????c6c5c4c3c2c1
$last_quad:
	negq	$18, $6		#-e0	:
        xor	$17, $1, $1	# .. e1 :
	srl	$3, $6, $6	# e0    : $6 = mask of $18 bits set
        cmpbge  $31, $1, $2	# .. e1 :
	and	$2, $6, $2	#-e0	:
        beq     $2, $not_found	# .. e1 :

$found_it:
	# Now, determine which byte matched:
        negq    $2, $3		# e0	:
        and     $2, $3, $2	# e1	:

        and     $2, 0x0f, $1	#-e0	:
        addq    $0, 4, $3	# .. e1 :
        cmoveq  $1, $3, $0	# e0	:

        addq    $0, 2, $3	# .. e1 :
        and     $2, 0x33, $1	#-e0	:
        cmoveq  $1, $3, $0	# .. e1 :

        and     $2, 0x55, $1	# e0	:
        addq    $0, 1, $3	# .. e1 :
        cmoveq  $1, $3, $0	#-e0	:

$done:	ret			# .. e1 :

	# Deal with the case where $18 > 8 bytes remain to be
	# searched.  $16 may not be aligned.
	.align 4
$first_quad:
	andnot	$16, 0x7, $0	#-e1	:
        insqh   $3, $16, $2	# .. e0	: $2 = 0000ffffffffffff ($16<0:2> ff)
        xor	$1, $17, $1	# e0	:
	or	$1, $2, $1	# e1	: $1 = ====ffffffffffff
        cmpbge  $31, $1, $2	#-e0	:
        bne     $2, $found_it	# .. e1 :

	# At least one byte left to process.

	ldq	$1, 8($0)	# e0	:
	subq	$5, 1, $18	# .. e1 :
	addq	$0, 8, $0	#-e0	:

	# Make $18 point to last quad to be accessed (the
	# last quad may or may not be partial).

	andnot	$18, 0x7, $18	# .. e1 :
	cmpult	$0, $18, $2	# e0	:
	beq	$2, $final	# .. e1 :

	# At least two quads remain to be accessed.

	subq	$18, $0, $4	#-e0	: $4 <- nr quads to be processed
	and	$4, 8, $4	# e1	: odd number of quads?
	bne	$4, $odd_quad_count # e1 :

	# At least three quads remain to be accessed

	mov	$1, $4		# e0	: move prefetched value to correct reg

	.align	4
$unrolled_loop:
	ldq	$1, 8($0)	#-e0	: prefetch $1
	xor	$17, $4, $2	# .. e1 :
	cmpbge	$31, $2, $2	# e0	:
	bne	$2, $found_it	# .. e1 :

	addq	$0, 8, $0	#-e0	:
$odd_quad_count:
	xor	$17, $1, $2	# .. e1 :
	ldq	$4, 8($0)	# e0	: prefetch $4
	cmpbge	$31, $2, $2	# .. e1 :
	addq	$0, 8, $6	#-e0	:
	bne	$2, $found_it	# .. e1	:

	cmpult	$6, $18, $6	# e0	:
	addq	$0, 8, $0	# .. e1 :
	bne	$6, $unrolled_loop #-e1 :

	mov	$4, $1		# e0	: move prefetched value into $1
$final:	subq	$5, $0, $18	# .. e1	: $18 <- number of bytes left to do
	bne	$18, $last_quad	# e1	:

$not_found:
	mov	$31, $0		#-e0	:
	ret			# .. e1 :

        .end memchr
