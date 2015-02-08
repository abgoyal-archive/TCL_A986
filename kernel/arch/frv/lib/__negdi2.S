


        .text
        .p2align	4

###############################################################################
#
# unsigned long long __negdi2(unsigned long long value [GR8:GR9])
#
###############################################################################
        .globl		__negdi2
        .type		__negdi2,@function
__negdi2:
	subcc		gr0,gr9,gr9,icc0
	subx		gr0,gr8,gr8,icc0
	bralr
	.size		__negdi2, .-__negdi2

