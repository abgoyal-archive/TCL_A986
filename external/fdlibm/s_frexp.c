
/* @(#)s_frexp.c 1.4 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
static const double
#else
static double
#endif
two54 =  1.80143985094819840000e+16; /* 0x43500000, 0x00000000 */

#ifdef __STDC__
	double ieee_frexp(double x, int *eptr)
#else
	double ieee_frexp(x, eptr)
	double x; int *eptr;
#endif
{
	int  hx, ix, lx;
	hx = __HI(x);
	ix = 0x7fffffff&hx;
	lx = __LO(x);
	*eptr = 0;
	if(ix>=0x7ff00000||((ix|lx)==0)) return x;	/* 0,inf,nan */
	if (ix<0x00100000) {		/* subnormal */
	    x *= two54;
	    hx = __HI(x);
	    ix = hx&0x7fffffff;
	    *eptr = -54;
	}
	*eptr += (ix>>20)-1022;
	hx = (hx&0x800fffff)|0x3fe00000;
	__HI(x) = hx;
	return x;
}
