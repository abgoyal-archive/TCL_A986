
/* @(#)w_scalb.c 1.3 95/01/18 */


#include "fdlibm.h"

#include <errno.h>

#ifdef __STDC__
#ifdef _SCALB_INT
	double ieee_scalb(double x, int fn)		/* wrapper scalb */
#else
	double ieee_scalb(double x, double fn)	/* wrapper scalb */
#endif
#else
	double ieee_scalb(x,fn)			/* wrapper scalb */
#ifdef _SCALB_INT
	double x; int fn;
#else
	double x,fn;
#endif
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_scalb(x,fn);
#else
	double z;
	z = __ieee754_scalb(x,fn);
	if(_LIB_VERSION == _IEEE_) return z;
	if(!(ieee_finite(z)||ieee_isnan(z))&&ieee_finite(x)) {
	    return __kernel_standard(x,(double)fn,32); /* scalb overflow */
	}
	if(z==0.0&&z!=x) {
	    return __kernel_standard(x,(double)fn,33); /* scalb underflow */
	} 
#ifndef _SCALB_INT
	if(!ieee_finite(fn)) errno = ERANGE;
#endif
	return z;
#endif 
}
