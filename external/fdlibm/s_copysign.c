
/* @(#)s_copysign.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_copysign(double x, double y)
#else
	double ieee_copysign(x,y)
	double x,y;
#endif
{
	__HI(x) = (__HI(x)&0x7fffffff)|(__HI(y)&0x80000000);
        return x;
}
