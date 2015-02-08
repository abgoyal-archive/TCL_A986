

/* Allow an exception for an insn; exit if we get one.  */
#define EX(x,y...)			\
	99: x,##y;			\
	.section __ex_table,"a";	\
	.long 99b - .;			\
	lda $31, $exception-99b($31); 	\
	.previous

	.set noat
	.set noreorder
	.align 4

	.globl __do_clear_user
	.ent __do_clear_user
	.frame	$30, 0, $28
	.prologue 0

				# Pipeline info : Slotting & Comments
__do_clear_user:
	and	$6, 7, $4	# .. E  .. ..	: find dest head misalignment
	beq	$0, $zerolength # U  .. .. ..	:  U L U L

	addq	$0, $4, $1	# .. .. .. E	: bias counter
	and	$1, 7, $2	# .. .. E  ..	: number of misaligned bytes in tail
# Note - we never actually use $2, so this is a moot computation
# and we can rewrite this later...
	srl	$1, 3, $1	# .. E  .. ..	: number of quadwords to clear
	beq	$4, $headalign	# U  .. .. ..	: U L U L

	EX( ldq_u $5, 0($6) )	# .. .. .. L	: load dst word to mask back in
	beq	$1, $onebyte	# .. .. U  ..	: sub-word store?
	mskql	$5, $6, $5	# .. U  .. ..	: take care of misaligned head
	addq	$6, 8, $6	# E  .. .. .. 	: L U U L

	EX( stq_u $5, -8($6) )	# .. .. .. L	:
	subq	$1, 1, $1	# .. .. E  ..	:
	addq	$0, $4, $0	# .. E  .. ..	: bytes left -= 8 - misalignment
	subq	$0, 8, $0	# E  .. .. ..	: U L U L

	.align	4
$headalign:
	subq	$1, 16, $4	# .. .. .. E	: If < 16, we can not use the huge loop
	and	$6, 0x3f, $2	# .. .. E  ..	: Forward work for huge loop
	subq	$2, 0x40, $3	# .. E  .. ..	: bias counter (huge loop)
	blt	$4, $trailquad	# U  .. .. ..	: U L U L


	nop			# .. .. .. E
	nop			# .. .. E  ..
	nop			# .. E  .. ..
	beq	$3, $bigalign	# U  .. .. ..	: U L U L : Aligned 0mod64

$alignmod64:
	EX( stq_u $31, 0($6) )	# .. .. .. L
	addq	$3, 8, $3	# .. .. E  ..
	subq	$0, 8, $0	# .. E  .. ..
	nop			# E  .. .. ..	: U L U L

	nop			# .. .. .. E
	subq	$1, 1, $1	# .. .. E  ..
	addq	$6, 8, $6	# .. E  .. ..
	blt	$3, $alignmod64	# U  .. .. ..	: U L U L

$bigalign:
	nop			# E :
	nop			# E :
	nop			# E :
	bis	$6,$6,$3	# E : U L U L : Initial wh64 address is dest
	/* This might actually help for the current trip... */

$do_wh64:
	wh64	($3)		# .. .. .. L1	: memory subsystem hint
	subq	$1, 16, $4	# .. .. E  ..	: Forward calculation - repeat the loop?
	EX( stq_u $31, 0($6) )	# .. L  .. ..
	subq	$0, 8, $0	# E  .. .. ..	: U L U L

	addq	$6, 128, $3	# E : Target address of wh64
	EX( stq_u $31, 8($6) )	# L :
	EX( stq_u $31, 16($6) )	# L :
	subq	$0, 16, $0	# E : U L L U

	nop			# E :
	EX( stq_u $31, 24($6) )	# L :
	EX( stq_u $31, 32($6) )	# L :
	subq	$0, 168, $5	# E : U L L U : two trips through the loop left?
	/* 168 = 192 - 24, since we've already completed some stores */

	subq	$0, 16, $0	# E :
	EX( stq_u $31, 40($6) )	# L :
	EX( stq_u $31, 48($6) )	# L :
	cmovlt	$5, $6, $3	# E : U L L U : Latency 2, extra mapping cycle

	subq	$1, 8, $1	# E :
	subq	$0, 16, $0	# E :
	EX( stq_u $31, 56($6) )	# L :
	nop			# E : U L U L

	nop			# E :
	subq	$0, 8, $0	# E :
	addq	$6, 64, $6	# E :
	bge	$4, $do_wh64	# U : U L U L

$trailquad:
	# zero to 16 quadwords left to store, plus any trailing bytes
	# $1 is the number of quadwords left to go.
	# 
	nop			# .. .. .. E
	nop			# .. .. E  ..
	nop			# .. E  .. ..
	beq	$1, $trailbytes	# U  .. .. ..	: U L U L : Only 0..7 bytes to go

$onequad:
	EX( stq_u $31, 0($6) )	# .. .. .. L
	subq	$1, 1, $1	# .. .. E  ..
	subq	$0, 8, $0	# .. E  .. ..
	nop			# E  .. .. ..	: U L U L

	nop			# .. .. .. E
	nop			# .. .. E  ..
	addq	$6, 8, $6	# .. E  .. ..
	bgt	$1, $onequad	# U  .. .. ..	: U L U L

	# We have an unknown number of bytes left to go.
$trailbytes:
	nop			# .. .. .. E
	nop			# .. .. E  ..
	nop			# .. E  .. ..
	beq	$0, $zerolength	# U  .. .. ..	: U L U L

	# $0 contains the number of bytes left to copy (0..31)
	# so we will use $0 as the loop counter
	# We know for a fact that $0 > 0 zero due to previous context
$onebyte:
	EX( stb $31, 0($6) )	# .. .. .. L
	subq	$0, 1, $0	# .. .. E  ..	:
	addq	$6, 1, $6	# .. E  .. ..	:
	bgt	$0, $onebyte	# U  .. .. ..	: U L U L

$zerolength:
$exception:			# Destination for exception recovery(?)
	nop			# .. .. .. E	:
	nop			# .. .. E  ..	:
	nop			# .. E  .. ..	:
	ret	$31, ($28), 1	# L0 .. .. ..	: L U L U
	.end __do_clear_user

