
/* @(#)w_asin.c 1.3 95/01/18 */



#include "fdlibm.h"


#ifdef __STDC__
	double ieee_asin(double x)		/* wrapper asin */
#else
	double ieee_asin(x)			/* wrapper asin */
	double x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_asin(x);
#else
	double z;
	z = __ieee754_asin(x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x)) return z;
	if(ieee_fabs(x)>1.0) {
	        return __kernel_standard(x,x,2); /* ieee_asin(|x|>1) */
	} else
	    return z;
#endif
}
