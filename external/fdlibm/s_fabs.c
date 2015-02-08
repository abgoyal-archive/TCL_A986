
/* @(#)s_fabs.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_fabs(double x)
#else
	double ieee_fabs(x)
	double x;
#endif
{
	__HI(x) &= 0x7fffffff;
        return x;
}
