


        .text
        .p2align	4

###############################################################################
#
# void __insl_sw(unsigned int port, void *buf, int n)
#
###############################################################################
        .globl		__insl_sw
        .type		__insl_sw,@function
__insl_sw:
	subi.p		gr9,#1,gr9
	setlos		#1,gr4
0:
	ldi.p		@(gr8,#0),gr5		; get 0xAABBCCDD
	subicc		gr10,#1,gr10,icc0

	stbu.p		gr5,@(gr9,gr4)		; write 0xDD
	srli		gr5,#8,gr5
	stbu.p		gr5,@(gr9,gr4)		; write 0xCC
	srli		gr5,#8,gr5
	stbu.p		gr5,@(gr9,gr4)		; write 0xBB
	srli		gr5,#8,gr5
	stbu.p		gr5,@(gr9,gr4)		; write 0xAA
	bhi		icc0,#2,0b
	bralr

	.size		__insl_sw, .-__insl_sw
