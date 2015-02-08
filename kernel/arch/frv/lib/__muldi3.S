

        .text
        .p2align	4

###############################################################################
#
# unsigned long long __muldi3(unsigned long long x [GR8:GR9],
#                             unsigned long long y [GR10:GR11])
#
###############################################################################
        .globl		__muldi3, __mulll, __umulll
        .type		__muldi3,@function
__muldi3:
__mulll:
__umulll:
	umul		gr8,gr11,gr4		; GR4:GR5 = x.MSW * y.LSW
	umul		gr9,gr10,gr6		; GR6:GR7 = x.LSW * y.MSW
	umul.p		gr9,gr11,gr8		; GR8:GR9 = x.LSW * y.LSW
	add		gr5,gr7,gr5
	add.p		gr8,gr5,gr8		; GR8 += GR5 + GR7
	bralr
	.size		__muldi3, .-__muldi3
