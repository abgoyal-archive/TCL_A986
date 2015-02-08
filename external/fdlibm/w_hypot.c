
/* @(#)w_hypot.c 1.3 95/01/18 */


#include "fdlibm.h"


#ifdef __STDC__
	double ieee_hypot(double x, double y)/* wrapper hypot */
#else
	double ieee_hypot(x,y)		/* wrapper hypot */
	double x,y;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_hypot(x,y);
#else
	double z;
	z = __ieee754_hypot(x,y);
	if(_LIB_VERSION == _IEEE_) return z;
	if((!ieee_finite(z))&&ieee_finite(x)&&ieee_finite(y))
	    return __kernel_standard(x,y,4); /* hypot overflow */
	else
	    return z;
#endif
}
