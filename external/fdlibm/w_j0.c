
/* @(#)w_j0.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_j0(double x)		/* wrapper j0 */
#else
	double ieee_j0(x)			/* wrapper j0 */
	double x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_j0(x);
#else
	double z = __ieee754_j0(x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x)) return z;
	if(ieee_fabs(x)>X_TLOSS) {
	        return __kernel_standard(x,x,34); /* ieee_j0(|x|>X_TLOSS) */
	} else
	    return z;
#endif
}

#ifdef __STDC__
	double ieee_y0(double x)		/* wrapper y0 */
#else
	double ieee_y0(x)			/* wrapper y0 */
	double x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_y0(x);
#else
	double z;
	z = __ieee754_y0(x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x) ) return z;
        if(x <= 0.0){
                if(x==0.0)
                    /* d= -one/(x-x); */
                    return __kernel_standard(x,x,8);
                else
                    /* d = zero/(x-x); */
                    return __kernel_standard(x,x,9);
        }
	if(x>X_TLOSS) {
	        return __kernel_standard(x,x,35); /* ieee_y0(x>X_TLOSS) */
	} else
	    return z;
#endif
}
