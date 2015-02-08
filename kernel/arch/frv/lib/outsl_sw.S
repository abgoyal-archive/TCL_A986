


        .text
        .p2align	4

###############################################################################
#
# void __outsl_sw(unsigned int port, const void *buf, int n)
#
###############################################################################
        .globl		__outsl_sw
        .type		__outsl_sw,@function
__outsl_sw:
	subi.p		gr9,#1,gr9
	setlos		#1,gr4
0:
	ldubu		@(gr9,gr4),gr5
	ldubu		@(gr9,gr4),gr6
	slli		gr6,#8,gr6
	ldubu.p		@(gr9,gr4),gr7
	or		gr5,gr6,gr5
	ldubu.p		@(gr9,gr4),gr6
	slli		gr7,#16,gr7
	slli.p		gr6,#24,gr6
	or		gr5,gr7,gr5
	subicc.p	gr10,#1,gr10,icc0
	or		gr5,gr6,gr5

	sti.p		gr5,@(gr8,#0)
	bhi		icc0,#2,0b

	membar
	bralr

	.size		__outsl_sw, .-__outsl_sw
