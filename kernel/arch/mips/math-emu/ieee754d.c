

#include <linux/kernel.h>
#include "ieee754.h"

#define DP_EBIAS	1023
#define DP_EMIN		(-1022)
#define DP_EMAX		1023
#define DP_FBITS	52

#define SP_EBIAS	127
#define SP_EMIN		(-126)
#define SP_EMAX		127
#define SP_FBITS	23

#define DP_MBIT(x)	((u64)1 << (x))
#define DP_HIDDEN_BIT	DP_MBIT(DP_FBITS)
#define DP_SIGN_BIT	DP_MBIT(63)


#define SP_MBIT(x)	((u32)1 << (x))
#define SP_HIDDEN_BIT	SP_MBIT(SP_FBITS)
#define SP_SIGN_BIT	SP_MBIT(31)


#define SPSIGN(sp)	(sp.parts.sign)
#define SPBEXP(sp)	(sp.parts.bexp)
#define SPMANT(sp)	(sp.parts.mant)

#define DPSIGN(dp)	(dp.parts.sign)
#define DPBEXP(dp)	(dp.parts.bexp)
#define DPMANT(dp)	(dp.parts.mant)

ieee754dp ieee754dp_dump(char *m, ieee754dp x)
{
	int i;

	printk("%s", m);
	printk("<%08x,%08x>\n", (unsigned) (x.bits >> 32),
	       (unsigned) x.bits);
	printk("\t=");
	switch (ieee754dp_class(x)) {
	case IEEE754_CLASS_QNAN:
	case IEEE754_CLASS_SNAN:
		printk("Nan %c", DPSIGN(x) ? '-' : '+');
		for (i = DP_FBITS - 1; i >= 0; i--)
			printk("%c", DPMANT(x) & DP_MBIT(i) ? '1' : '0');
		break;
	case IEEE754_CLASS_INF:
		printk("%cInfinity", DPSIGN(x) ? '-' : '+');
		break;
	case IEEE754_CLASS_ZERO:
		printk("%cZero", DPSIGN(x) ? '-' : '+');
		break;
	case IEEE754_CLASS_DNORM:
		printk("%c0.", DPSIGN(x) ? '-' : '+');
		for (i = DP_FBITS - 1; i >= 0; i--)
			printk("%c", DPMANT(x) & DP_MBIT(i) ? '1' : '0');
		printk("e%d", DPBEXP(x) - DP_EBIAS);
		break;
	case IEEE754_CLASS_NORM:
		printk("%c1.", DPSIGN(x) ? '-' : '+');
		for (i = DP_FBITS - 1; i >= 0; i--)
			printk("%c", DPMANT(x) & DP_MBIT(i) ? '1' : '0');
		printk("e%d", DPBEXP(x) - DP_EBIAS);
		break;
	default:
		printk("Illegal/Unknown IEEE754 value class");
	}
	printk("\n");
	return x;
}

ieee754sp ieee754sp_dump(char *m, ieee754sp x)
{
	int i;

	printk("%s=", m);
	printk("<%08x>\n", (unsigned) x.bits);
	printk("\t=");
	switch (ieee754sp_class(x)) {
	case IEEE754_CLASS_QNAN:
	case IEEE754_CLASS_SNAN:
		printk("Nan %c", SPSIGN(x) ? '-' : '+');
		for (i = SP_FBITS - 1; i >= 0; i--)
			printk("%c", SPMANT(x) & SP_MBIT(i) ? '1' : '0');
		break;
	case IEEE754_CLASS_INF:
		printk("%cInfinity", SPSIGN(x) ? '-' : '+');
		break;
	case IEEE754_CLASS_ZERO:
		printk("%cZero", SPSIGN(x) ? '-' : '+');
		break;
	case IEEE754_CLASS_DNORM:
		printk("%c0.", SPSIGN(x) ? '-' : '+');
		for (i = SP_FBITS - 1; i >= 0; i--)
			printk("%c", SPMANT(x) & SP_MBIT(i) ? '1' : '0');
		printk("e%d", SPBEXP(x) - SP_EBIAS);
		break;
	case IEEE754_CLASS_NORM:
		printk("%c1.", SPSIGN(x) ? '-' : '+');
		for (i = SP_FBITS - 1; i >= 0; i--)
			printk("%c", SPMANT(x) & SP_MBIT(i) ? '1' : '0');
		printk("e%d", SPBEXP(x) - SP_EBIAS);
		break;
	default:
		printk("Illegal/Unknown IEEE754 value class");
	}
	printk("\n");
	return x;
}
