
/* @(#)w_gamma.c 1.3 95/01/18 */


#include "fdlibm.h"

extern int signgam;

#ifdef __STDC__
	double ieee_gamma(double x)
#else
	double ieee_gamma(x)
	double x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_gamma_r(x,&signgam);
#else
        double y;
        y = __ieee754_gamma_r(x,&signgam);
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
