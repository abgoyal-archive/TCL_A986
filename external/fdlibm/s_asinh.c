
/* @(#)s_asinh.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
static const double 
#else
static double 
#endif
one =  1.00000000000000000000e+00, /* 0x3FF00000, 0x00000000 */
ln2 =  6.93147180559945286227e-01, /* 0x3FE62E42, 0xFEFA39EF */
huge=  1.00000000000000000000e+300; 

#ifdef __STDC__
	double asinh(double x)
#else
	double asinh(x)
	double x;
#endif
{	
	double t,w;
	int hx,ix;
	hx = __HI(x);
	ix = hx&0x7fffffff;
	if(ix>=0x7ff00000) return x+x;	/* x is inf or NaN */
	if(ix< 0x3e300000) {	/* |x|<2**-28 */
	    if(huge+x>one) return x;	/* return x inexact except 0 */
	} 
	if(ix>0x41b00000) {	/* |x| > 2**28 */
	    w = __ieee754_log(ieee_fabs(x))+ln2;
	} else if (ix>0x40000000) {	/* 2**28 > |x| > 2.0 */
	    t = ieee_fabs(x);
	    w = __ieee754_log(2.0*t+one/(ieee_sqrt(x*x+one)+t));
	} else {		/* 2.0 > |x| > 2**-28 */
	    t = x*x;
	    w =ieee_log1p(ieee_fabs(x)+t/(one+ieee_sqrt(one+t)));
	}
	if(hx>0) return w; else return -w;
}
