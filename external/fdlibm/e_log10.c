
/* @(#)e_log10.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
static const double
#else
static double
#endif
two54      =  1.80143985094819840000e+16, /* 0x43500000, 0x00000000 */
ivln10     =  4.34294481903251816668e-01, /* 0x3FDBCB7B, 0x1526E50E */
log10_2hi  =  3.01029995663611771306e-01, /* 0x3FD34413, 0x509F6000 */
log10_2lo  =  3.69423907715893078616e-13; /* 0x3D59FEF3, 0x11F12B36 */

static double zero   =  0.0;

#ifdef __STDC__
	double __ieee754_log10(double x)
#else
	double __ieee754_log10(x)
	double x;
#endif
{
	double y,z;
	int i,k,hx;
	unsigned lx;

	hx = __HI(x);	/* high word of x */
	lx = __LO(x);	/* low word of x */

        k=0;
        if (hx < 0x00100000) {                  /* x < 2**-1022  */
            if (((hx&0x7fffffff)|lx)==0)
                return -two54/zero;             /* ieee_log(+-0)=-inf */
            if (hx<0) return (x-x)/zero;        /* ieee_log(-#) = NaN */
            k -= 54; x *= two54; /* subnormal number, scale up x */
            hx = __HI(x);                /* high word of x */
        }
	if (hx >= 0x7ff00000) return x+x;
	k += (hx>>20)-1023;
	i  = ((unsigned)k&0x80000000)>>31;
        hx = (hx&0x000fffff)|((0x3ff-i)<<20);
        y  = (double)(k+i);
        __HI(x) = hx;
	z  = y*log10_2lo + ivln10*__ieee754_log(x);
	return  z+y*log10_2hi;
}
