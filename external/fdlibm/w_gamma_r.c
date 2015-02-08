
/* @(#)w_gamma_r.c 1.3 95/01/18 */


#include "fdlibm.h"


#ifdef __STDC__
	double ieee_gamma_r(double x, int *signgamp) /* wrapper lgamma_r */
#else
	double ieee_gamma_r(x,signgamp)              /* wrapper lgamma_r */
        double x; int *signgamp;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_gamma_r(x,signgamp);
#else
        double y;
        y = __ieee754_gamma_r(x,signgamp);
        if(_LIB_VERSION == _IEEE_) return y;
        if(!ieee_finite(y)&&ieee_finite(x)) {
            if(ieee_floor(x)==x&&x<=0.0)
                return __kernel_standard(x,x,41); /* gamma pole */
            else
                return __kernel_standard(x,x,40); /* gamma overflow */
        } else
            return y;
#endif
}             
