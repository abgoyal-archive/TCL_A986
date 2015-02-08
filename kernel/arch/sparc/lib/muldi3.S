

	.text
	.align 4
	.globl __muldi3
__muldi3:
	save  %sp, -104, %sp
	wr  %g0, %i1, %y
	sra  %i3, 0x1f, %g2
	and  %i1, %g2, %g2
	andcc  %g0, 0, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, %i3, %g1
	mulscc  %g1, 0, %g1
	add  %g1, %g2, %l2
	rd  %y, %o1
	mov  %o1, %l3
	mov  %i1, %o0
	call  .umul
	mov  %i2, %o1
	mov  %o0, %l0
	mov  %i0, %o0
	call  .umul
	mov  %i3, %o1
	add  %l0, %o0, %l0
	mov  %l2, %i0
	add  %l2, %l0, %i0
	ret 
	restore  %g0, %l3, %o1
