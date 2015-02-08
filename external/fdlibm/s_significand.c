
/* @(#)s_significand.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_significand(double x)
#else
	double ieee_significand(x)
	double x;
#endif
{
	return __ieee754_scalb(x,(double) -ieee_ilogb(x));
}
