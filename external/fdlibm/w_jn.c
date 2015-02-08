
/* @(#)w_jn.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_jn(int n, double x)	/* wrapper jn */
#else
	double ieee_jn(n,x)			/* wrapper jn */
	double x; int n;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_jn(n,x);
#else
	double z;
	z = __ieee754_jn(n,x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x) ) return z;
	if(ieee_fabs(x)>X_TLOSS) {
	    return __kernel_standard((double)n,x,38); /* ieee_jn(|x|>X_TLOSS,n) */
	} else
	    return z;
#endif
}

#ifdef __STDC__
	double ieee_yn(int n, double x)	/* wrapper yn */
#else
	double ieee_yn(n,x)			/* wrapper yn */
	double x; int n;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_yn(n,x);
#else
	double z;
	z = __ieee754_yn(n,x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x) ) return z;
        if(x <= 0.0){
                if(x==0.0)
                    /* d= -one/(x-x); */
                    return __kernel_standard((double)n,x,12);
                else
                    /* d = zero/(x-x); */
                    return __kernel_standard((double)n,x,13);
        }
	if(x>X_TLOSS) {
	    return __kernel_standard((double)n,x,39); /* ieee_yn(x>X_TLOSS,n) */
	} else
	    return z;
#endif
}
