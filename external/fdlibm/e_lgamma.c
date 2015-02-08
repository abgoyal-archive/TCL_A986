
/* @(#)e_lgamma.c 1.3 95/01/18 */


#include "fdlibm.h"

extern int signgam;

#ifdef __STDC__
	double __ieee754_lgamma(double x)
#else
	double __ieee754_lgamma(x)
	double x;
#endif
{
	return __ieee754_lgamma_r(x,&signgam);
}
