
/* @(#)s_logb.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_logb(double x)
#else
	double ieee_logb(x)
	double x;
#endif
{
	int lx,ix;
	ix = (__HI(x))&0x7fffffff;	/* high |x| */
	lx = __LO(x);			/* low x */
	if((ix|lx)==0) return -1.0/ieee_fabs(x);
	if(ix>=0x7ff00000) return x*x;
	if((ix>>=20)==0) 			/* IEEE 754 logb */
		return -1022.0; 
	else
		return (double) (ix-1023); 
}
