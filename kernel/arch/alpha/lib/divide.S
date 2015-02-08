



#define halt .long 0

#define mask	$0
#define divisor	$1
#define compare $28
#define tmp1	$3
#define tmp2	$4

#ifdef DIV
#define DIV_ONLY(x,y...) x,##y
#define MOD_ONLY(x,y...)
#define func(x) __div##x
#define modulus $2
#define quotient $27
#define GETSIGN(x) xor $24,$25,x
#define STACK 48
#else
#define DIV_ONLY(x,y...)
#define MOD_ONLY(x,y...) x,##y
#define func(x) __rem##x
#define modulus $27
#define quotient $2
#define GETSIGN(x) bis $24,$24,x
#define STACK 32
#endif

#ifdef INTSIZE
#define ufunction func(lu)
#define sfunction func(l)
#define LONGIFY(x) zapnot x,15,x
#define SLONGIFY(x) addl x,0,x
#else
#define ufunction func(qu)
#define sfunction func(q)
#define LONGIFY(x)
#define SLONGIFY(x)
#endif

.set noat
.align	3
.globl	ufunction
.ent	ufunction
ufunction:
	subq	$30,STACK,$30
	.frame	$30,STACK,$23
	.prologue 0

7:	stq	$1, 0($30)
	bis	$25,$25,divisor
	stq	$2, 8($30)
	bis	$24,$24,modulus
	stq	$0,16($30)
	bis	$31,$31,quotient
	LONGIFY(divisor)
	stq	tmp1,24($30)
	LONGIFY(modulus)
	bis	$31,1,mask
	DIV_ONLY(stq tmp2,32($30))
	beq	divisor, 9f			/* div by zero */

#ifdef INTSIZE
	/*
	 * shift divisor left, using 3-bit shifts for
	 * 32-bit divides as we can't overflow. Three-bit
	 * shifts will result in looping three times less
	 * here, but can result in two loops more later.
	 * Thus using a large shift isn't worth it (and
	 * s8add pairs better than a sll..)
	 */
1:	cmpult	divisor,modulus,compare
	s8addq	divisor,$31,divisor
	s8addq	mask,$31,mask
	bne	compare,1b
#else
1:	cmpult	divisor,modulus,compare
	blt     divisor, 2f
	addq	divisor,divisor,divisor
	addq	mask,mask,mask
	bne	compare,1b
	unop
#endif

	/* ok, start to go right again.. */
2:	DIV_ONLY(addq quotient,mask,tmp2)
	srl	mask,1,mask
	cmpule	divisor,modulus,compare
	subq	modulus,divisor,tmp1
	DIV_ONLY(cmovne compare,tmp2,quotient)
	srl	divisor,1,divisor
	cmovne	compare,tmp1,modulus
	bne	mask,2b

9:	ldq	$1, 0($30)
	ldq	$2, 8($30)
	ldq	$0,16($30)
	ldq	tmp1,24($30)
	DIV_ONLY(ldq tmp2,32($30))
	addq	$30,STACK,$30
	ret	$31,($23),1
	.end	ufunction

.align 3
.globl	sfunction
.ent	sfunction
sfunction:
	subq	$30,STACK,$30
	.frame	$30,STACK,$23
	.prologue 0
	bis	$24,$25,$28
	SLONGIFY($28)
	bge	$28,7b
	stq	$24,0($30)
	subq	$31,$24,$28
	stq	$25,8($30)
	cmovlt	$24,$28,$24	/* abs($24) */
	stq	$23,16($30)
	subq	$31,$25,$28
	stq	tmp1,24($30)
	cmovlt	$25,$28,$25	/* abs($25) */
	unop
	bsr	$23,ufunction
	ldq	$24,0($30)
	ldq	$25,8($30)
	GETSIGN($28)
	subq	$31,$27,tmp1
	SLONGIFY($28)
	ldq	$23,16($30)
	cmovlt	$28,tmp1,$27
	ldq	tmp1,24($30)
	addq	$30,STACK,$30
	ret	$31,($23),1
	.end	sfunction
