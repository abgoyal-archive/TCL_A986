


	.balign 4
	.global	__udivsi3_i4i
	.global	__udivsi3_i4
	.set	__udivsi3_i4, __udivsi3_i4i
	.type	__udivsi3_i4i, @function
	.type	__sdivsi3_i4i, @function
__udivsi3_i4i:
	sts pr,r1
	mov.l r4,@-r15
	extu.w r5,r0
	cmp/eq r5,r0
	swap.w r4,r0
	shlr16 r4
	bf/s large_divisor
	div0u
	mov.l r5,@-r15
	shll16 r5
sdiv_small_divisor:
	div1 r5,r4
	bsr div6
	div1 r5,r4
	div1 r5,r4
	bsr div6
	div1 r5,r4
	xtrct r4,r0
	xtrct r0,r4
	bsr div7
	swap.w r4,r4
	div1 r5,r4
	bsr div7
	div1 r5,r4
	xtrct r4,r0
	mov.l @r15+,r5
	swap.w r0,r0
	mov.l @r15+,r4
	jmp @r1
	rotcl r0
div7:
	div1 r5,r4
div6:
	            div1 r5,r4; div1 r5,r4; div1 r5,r4
	div1 r5,r4; div1 r5,r4; rts;        div1 r5,r4

divx3:
	rotcl r0
	div1 r5,r4
	rotcl r0
	div1 r5,r4
	rotcl r0
	rts
	div1 r5,r4

large_divisor:
	mov.l r5,@-r15
sdiv_large_divisor:
	xor r4,r0
	.rept 4
	rotcl r0
	bsr divx3
	div1 r5,r4
	.endr
	mov.l @r15+,r5
	mov.l @r15+,r4
	jmp @r1
	rotcl r0

	.global	__sdivsi3_i4i
	.global __sdivsi3_i4
	.global __sdivsi3
	.set	__sdivsi3_i4, __sdivsi3_i4i
	.set	__sdivsi3, __sdivsi3_i4i
__sdivsi3_i4i:
	mov.l r4,@-r15
	cmp/pz r5
	mov.l r5,@-r15
	bt/s pos_divisor
	cmp/pz r4
	neg r5,r5
	extu.w r5,r0
	bt/s neg_result
	cmp/eq r5,r0
	neg r4,r4
pos_result:
	swap.w r4,r0
	bra sdiv_check_divisor
	sts pr,r1
pos_divisor:
	extu.w r5,r0
	bt/s pos_result
	cmp/eq r5,r0
	neg r4,r4
neg_result:
	mova negate_result,r0
	;
	mov r0,r1
	swap.w r4,r0
	lds r2,macl
	sts pr,r2
sdiv_check_divisor:
	shlr16 r4
	bf/s sdiv_large_divisor
	div0u
	bra sdiv_small_divisor
	shll16 r5
	.balign 4
negate_result:
	neg r0,r0
	jmp @r2
	sts macl,r2
