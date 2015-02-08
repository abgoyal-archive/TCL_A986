
/* @(#)s_isnan.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	int ieee_isnan(double x)
#else
	int ieee_isnan(x)
	double x;
#endif
{
	int hx,lx;
	hx = (__HI(x)&0x7fffffff);
	lx = __LO(x);
	hx |= (unsigned)(lx|(-lx))>>31;	
	hx = 0x7ff00000 - hx;
	return ((unsigned)(hx))>>31;
}
