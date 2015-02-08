
/* @(#)s_finite.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	int ieee_finite(double x)
#else
	int ieee_finite(x)
	double x;
#endif
{
	int hx; 
	hx = __HI(x);
	return  (unsigned)((hx&0x7fffffff)-0x7ff00000)>>31;
}
