



#include <variant/core.h>

#ifdef __XTENSA_EB__
#define ALIGN(R, W0, W1) src	R, W0, W1
#define SSA8(R)	ssa8b R
#else
#define ALIGN(R, W0, W1) src	R, W1, W0
#define SSA8(R)	ssa8l R
#endif

/* Load or store instructions that may cause exceptions use the EX macro. */

#define EX(insn,reg1,reg2,offset,handler)	\
9:	insn	reg1, reg2, offset;		\
	.section __ex_table, "a";		\
	.word	9b, handler;			\
	.previous


	.text
	.align	4
	.global	__xtensa_copy_user
	.type	__xtensa_copy_user,@function
__xtensa_copy_user:
	entry	sp, 16		# minimal stack frame
	# a2/ dst, a3/ src, a4/ len
	mov	a5, a2		# copy dst so that a2 is return value
	mov	a11, a4		# preserve original len for error case
.Lcommon:
	bbsi.l	a2, 0, .Ldst1mod2 # if dst is 1 mod 2
	bbsi.l	a2, 1, .Ldst2mod4 # if dst is 2 mod 4
.Ldstaligned:	# return here from .Ldstunaligned when dst is aligned
	srli	a7, a4, 4	# number of loop iterations with 16B
				# per iteration
	movi	a8, 3		  # if source is also aligned,
	bnone	a3, a8, .Laligned # then use word copy
	SSA8(	a3)		# set shift amount from byte offset
	bnez	a4, .Lsrcunaligned
	movi	a2, 0		# return success for len==0
	retw


.Ldst1mod2:	# dst is only byte aligned
	bltui	a4, 7, .Lbytecopy	# do short copies byte by byte

	# copy 1 byte
	EX(l8ui, a6, a3, 0, l_fixup)
	addi	a3, a3,  1
	EX(s8i, a6, a5,  0, s_fixup)
	addi	a5, a5,  1
	addi	a4, a4, -1
	bbci.l	a5, 1, .Ldstaligned	# if dst is now aligned, then
					# return to main algorithm
.Ldst2mod4:	# dst 16-bit aligned
	# copy 2 bytes
	bltui	a4, 6, .Lbytecopy	# do short copies byte by byte
	EX(l8ui, a6, a3, 0, l_fixup)
	EX(l8ui, a7, a3, 1, l_fixup)
	addi	a3, a3,  2
	EX(s8i, a6, a5,  0, s_fixup)
	EX(s8i, a7, a5,  1, s_fixup)
	addi	a5, a5,  2
	addi	a4, a4, -2
	j	.Ldstaligned	# dst is now aligned, return to main algorithm

	.align	4
	.byte	0		# 1 mod 4 alignment for LOOPNEZ
				# (0 mod 4 alignment for LBEG)
.Lbytecopy:
#if XCHAL_HAVE_LOOPS
	loopnez	a4, .Lbytecopydone
#else /* !XCHAL_HAVE_LOOPS */
	beqz	a4, .Lbytecopydone
	add	a7, a3, a4	# a7 = end address for source
#endif /* !XCHAL_HAVE_LOOPS */
.Lnextbyte:
	EX(l8ui, a6, a3, 0, l_fixup)
	addi	a3, a3, 1
	EX(s8i, a6, a5, 0, s_fixup)
	addi	a5, a5, 1
#if !XCHAL_HAVE_LOOPS
	blt	a3, a7, .Lnextbyte
#endif /* !XCHAL_HAVE_LOOPS */
.Lbytecopydone:
	movi	a2, 0		# return success for len bytes copied
	retw

	# copy 16 bytes per iteration for word-aligned dst and word-aligned src
	.align	4		# 1 mod 4 alignment for LOOPNEZ
	.byte	0		# (0 mod 4 alignment for LBEG)
.Laligned:
#if XCHAL_HAVE_LOOPS
	loopnez	a7, .Loop1done
#else /* !XCHAL_HAVE_LOOPS */
	beqz	a7, .Loop1done
	slli	a8, a7, 4
	add	a8, a8, a3	# a8 = end of last 16B source chunk
#endif /* !XCHAL_HAVE_LOOPS */
.Loop1:
	EX(l32i, a6, a3,  0, l_fixup)
	EX(l32i, a7, a3,  4, l_fixup)
	EX(s32i, a6, a5,  0, s_fixup)
	EX(l32i, a6, a3,  8, l_fixup)
	EX(s32i, a7, a5,  4, s_fixup)
	EX(l32i, a7, a3, 12, l_fixup)
	EX(s32i, a6, a5,  8, s_fixup)
	addi	a3, a3, 16
	EX(s32i, a7, a5, 12, s_fixup)
	addi	a5, a5, 16
#if !XCHAL_HAVE_LOOPS
	blt	a3, a8, .Loop1
#endif /* !XCHAL_HAVE_LOOPS */
.Loop1done:
	bbci.l	a4, 3, .L2
	# copy 8 bytes
	EX(l32i, a6, a3,  0, l_fixup)
	EX(l32i, a7, a3,  4, l_fixup)
	addi	a3, a3,  8
	EX(s32i, a6, a5,  0, s_fixup)
	EX(s32i, a7, a5,  4, s_fixup)
	addi	a5, a5,  8
.L2:
	bbci.l	a4, 2, .L3
	# copy 4 bytes
	EX(l32i, a6, a3,  0, l_fixup)
	addi	a3, a3,  4
	EX(s32i, a6, a5,  0, s_fixup)
	addi	a5, a5,  4
.L3:
	bbci.l	a4, 1, .L4
	# copy 2 bytes
	EX(l16ui, a6, a3,  0, l_fixup)
	addi	a3, a3,  2
	EX(s16i,  a6, a5,  0, s_fixup)
	addi	a5, a5,  2
.L4:
	bbci.l	a4, 0, .L5
	# copy 1 byte
	EX(l8ui, a6, a3,  0, l_fixup)
	EX(s8i,  a6, a5,  0, s_fixup)
.L5:
	movi	a2, 0		# return success for len bytes copied
	retw


	.align	4
	.byte	0		# 1 mod 4 alignement for LOOPNEZ
				# (0 mod 4 alignment for LBEG)
.Lsrcunaligned:
	# copy 16 bytes per iteration for word-aligned dst and unaligned src
	and	a10, a3, a8	# save unalignment offset for below
	sub	a3, a3, a10	# align a3 (to avoid sim warnings only; not needed for hardware)
	EX(l32i, a6, a3, 0, l_fixup)	# load first word
#if XCHAL_HAVE_LOOPS
	loopnez	a7, .Loop2done
#else /* !XCHAL_HAVE_LOOPS */
	beqz	a7, .Loop2done
	slli	a10, a7, 4
	add	a10, a10, a3	# a10 = end of last 16B source chunk
#endif /* !XCHAL_HAVE_LOOPS */
.Loop2:
	EX(l32i, a7, a3,  4, l_fixup)
	EX(l32i, a8, a3,  8, l_fixup)
	ALIGN(	a6, a6, a7)
	EX(s32i, a6, a5,  0, s_fixup)
	EX(l32i, a9, a3, 12, l_fixup)
	ALIGN(	a7, a7, a8)
	EX(s32i, a7, a5,  4, s_fixup)
	EX(l32i, a6, a3, 16, l_fixup)
	ALIGN(	a8, a8, a9)
	EX(s32i, a8, a5,  8, s_fixup)
	addi	a3, a3, 16
	ALIGN(	a9, a9, a6)
	EX(s32i, a9, a5, 12, s_fixup)
	addi	a5, a5, 16
#if !XCHAL_HAVE_LOOPS
	blt	a3, a10, .Loop2
#endif /* !XCHAL_HAVE_LOOPS */
.Loop2done:
	bbci.l	a4, 3, .L12
	# copy 8 bytes
	EX(l32i, a7, a3,  4, l_fixup)
	EX(l32i, a8, a3,  8, l_fixup)
	ALIGN(	a6, a6, a7)
	EX(s32i, a6, a5,  0, s_fixup)
	addi	a3, a3,  8
	ALIGN(	a7, a7, a8)
	EX(s32i, a7, a5,  4, s_fixup)
	addi	a5, a5,  8
	mov	a6, a8
.L12:
	bbci.l	a4, 2, .L13
	# copy 4 bytes
	EX(l32i, a7, a3,  4, l_fixup)
	addi	a3, a3,  4
	ALIGN(	a6, a6, a7)
	EX(s32i, a6, a5,  0, s_fixup)
	addi	a5, a5,  4
	mov	a6, a7
.L13:
	add	a3, a3, a10	# readjust a3 with correct misalignment
	bbci.l	a4, 1, .L14
	# copy 2 bytes
	EX(l8ui, a6, a3,  0, l_fixup)
	EX(l8ui, a7, a3,  1, l_fixup)
	addi	a3, a3,  2
	EX(s8i, a6, a5,  0, s_fixup)
	EX(s8i, a7, a5,  1, s_fixup)
	addi	a5, a5,  2
.L14:
	bbci.l	a4, 0, .L15
	# copy 1 byte
	EX(l8ui, a6, a3,  0, l_fixup)
	EX(s8i,  a6, a5,  0, s_fixup)
.L15:
	movi	a2, 0		# return success for len bytes copied
	retw


	.section .fixup, "ax"
	.align	4


.Lmemset:
	.word	memset

s_fixup:
	sub	a2, a5, a2	/* a2 <-- bytes copied */
	sub	a2, a11, a2	/* a2 <-- bytes not copied */
	retw

l_fixup:
	sub	a2, a5, a2	/* a2 <-- bytes copied */
	sub	a2, a11, a2	/* a2 <-- bytes not copied == return value */

	/* void *memset(void *s, int c, size_t n); */
	mov	a6, a5		/* s */
	movi	a7, 0		/* c */
	mov	a8, a2		/* n */
	l32r	a4, .Lmemset
	callx4	a4
	/* Ignore memset return value in a6. */
	/* a2 still contains bytes not copied. */
	retw

