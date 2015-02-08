
/* @(#)s_expm1.c 1.5 04/04/22 */


#include "fdlibm.h"

#ifdef __STDC__
static const double
#else
static double
#endif
one		= 1.0,
huge		= 1.0e+300,
tiny		= 1.0e-300,
o_threshold	= 7.09782712893383973096e+02,/* 0x40862E42, 0xFEFA39EF */
ln2_hi		= 6.93147180369123816490e-01,/* 0x3fe62e42, 0xfee00000 */
ln2_lo		= 1.90821492927058770002e-10,/* 0x3dea39ef, 0x35793c76 */
invln2		= 1.44269504088896338700e+00,/* 0x3ff71547, 0x652b82fe */
	/* scaled coefficients related to expm1 */
Q1  =  -3.33333333333331316428e-02, /* BFA11111 111110F4 */
Q2  =   1.58730158725481460165e-03, /* 3F5A01A0 19FE5585 */
Q3  =  -7.93650757867487942473e-05, /* BF14CE19 9EAADBB7 */
Q4  =   4.00821782732936239552e-06, /* 3ED0CFCA 86E65239 */
Q5  =  -2.01099218183624371326e-07; /* BE8AFDB7 6E09C32D */

#ifdef __STDC__
	double ieee_expm1(double x)
#else
	double ieee_expm1(x)
	double x;
#endif
{
	double y,hi,lo,c,t,e,hxs,hfx,r1;
	int k,xsb;
	unsigned hx;

	hx  = __HI(x);	/* high word of x */
	xsb = hx&0x80000000;		/* sign bit of x */
	if(xsb==0) y=x; else y= -x;	/* y = |x| */
	hx &= 0x7fffffff;		/* high word of |x| */

    /* filter out huge and non-finite argument */
	if(hx >= 0x4043687A) {			/* if |x|>=56*ln2 */
	    if(hx >= 0x40862E42) {		/* if |x|>=709.78... */
                if(hx>=0x7ff00000) {
		    if(((hx&0xfffff)|__LO(x))!=0) 
		         return x+x; 	 /* NaN */
		    else return (xsb==0)? x:-1.0;/* ieee_exp(+-inf)={inf,-1} */
	        }
	        if(x > o_threshold) return huge*huge; /* overflow */
	    }
	    if(xsb!=0) { /* x < -56*ln2, return -1.0 with inexact */
		if(x+tiny<0.0)		/* raise inexact */
		return tiny-one;	/* return -1 */
	    }
	}

    /* argument reduction */
	if(hx > 0x3fd62e42) {		/* if  |x| > 0.5 ln2 */ 
	    if(hx < 0x3FF0A2B2) {	/* and |x| < 1.5 ln2 */
		if(xsb==0)
		    {hi = x - ln2_hi; lo =  ln2_lo;  k =  1;}
		else
		    {hi = x + ln2_hi; lo = -ln2_lo;  k = -1;}
	    } else {
		k  = invln2*x+((xsb==0)?0.5:-0.5);
		t  = k;
		hi = x - t*ln2_hi;	/* t*ln2_hi is exact here */
		lo = t*ln2_lo;
	    }
	    x  = hi - lo;
	    c  = (hi-x)-lo;
	} 
	else if(hx < 0x3c900000) {  	/* when |x|<2**-54, return x */
	    // t = huge+x;	/* return x with inexact flags when x!=0 */
	    // return x - (t-(huge+x));	
	    return x;	// inexact flag is not set, but Java ignors this flag anyway
	}
	else k = 0;

    /* x is now in primary range */
	hfx = 0.5*x;
	hxs = x*hfx;
	r1 = one+hxs*(Q1+hxs*(Q2+hxs*(Q3+hxs*(Q4+hxs*Q5))));
	t  = 3.0-r1*hfx;
	e  = hxs*((r1-t)/(6.0 - x*t));
	if(k==0) return x - (x*e-hxs);		/* c is 0 */
	else {
	    e  = (x*(e-c)-c);
	    e -= hxs;
	    if(k== -1) return 0.5*(x-e)-0.5;
	    if(k==1) 
	       	if(x < -0.25) return -2.0*(e-(x+0.5));
	       	else 	      return  one+2.0*(x-e);
	    if (k <= -2 || k>56) {   /* suffice to return ieee_exp(x)-1 */
	        y = one-(e-x);
	        __HI(y) += (k<<20);	/* add k to y's exponent */
	        return y-one;
	    }
	    t = one;
	    if(k<20) {
	       	__HI(t) = 0x3ff00000 - (0x200000>>k);  /* t=1-2^-k */
	       	y = t-(e-x);
	       	__HI(y) += (k<<20);	/* add k to y's exponent */
	   } else {
	       	__HI(t)  = ((0x3ff-k)<<20);	/* 2^-k */
	       	y = x-(e+t);
	       	y += one;
	       	__HI(y) += (k<<20);	/* add k to y's exponent */
	    }
	}
	return y;
}
