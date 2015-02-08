
/* @(#)e_atanh.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
static const double one = 1.0, huge = 1e300;
#else
static double one = 1.0, huge = 1e300;
#endif

static double zero = 0.0;

#ifdef __STDC__
	double __ieee754_atanh(double x)
#else
	double __ieee754_atanh(x)
	double x;
#endif
{
	double t;
	int hx,ix;
	unsigned lx;
	hx = __HI(x);		/* high word */
	lx = __LO(x);		/* low word */
	ix = hx&0x7fffffff;
	if ((ix|((lx|(-lx))>>31))>0x3ff00000) /* |x|>1 */
	    return (x-x)/(x-x);
	if(ix==0x3ff00000) 
	    return x/zero;
	if(ix<0x3e300000&&(huge+x)>zero) return x;	/* x<2**-28 */
	__HI(x) = ix;		/* x <- |x| */
	if(ix<0x3fe00000) {		/* x < 0.5 */
	    t = x+x;
	    t = 0.5*ieee_log1p(t+t*x/(one-x));
	} else 
	    t = 0.5*ieee_log1p((x+x)/(one-x));
	if(hx>=0) return t; else return -t;
}
