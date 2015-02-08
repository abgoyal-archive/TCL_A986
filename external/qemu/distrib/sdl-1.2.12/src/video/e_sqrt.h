
/* @(#)e_sqrt.c 5.1 93/09/24 */

#if defined(LIBM_SCCS) && !defined(lint)
static char rcsid[] = "$NetBSD: e_sqrt.c,v 1.8 1995/05/10 20:46:17 jtc Exp $";
#endif


/*#include "math.h"*/
#include "math_private.h"

#ifdef __STDC__
	double SDL_NAME(copysign)(double x, double y)
#else
	double SDL_NAME(copysign)(x,y)
	double x,y;
#endif
{
	u_int32_t hx,hy;
	GET_HIGH_WORD(hx,x);
	GET_HIGH_WORD(hy,y);
	SET_HIGH_WORD(x,(hx&0x7fffffff)|(hy&0x80000000));
        return x;
}

#ifdef __STDC__
	double SDL_NAME(scalbn) (double x, int n)
#else
	double SDL_NAME(scalbn) (x,n)
	double x; int n;
#endif
{
	int32_t k,hx,lx;
	EXTRACT_WORDS(hx,lx,x);
        k = (hx&0x7ff00000)>>20;		/* extract exponent */
        if (k==0) {				/* 0 or subnormal x */
            if ((lx|(hx&0x7fffffff))==0) return x; /* +-0 */
	    x *= two54;
	    GET_HIGH_WORD(hx,x);
	    k = ((hx&0x7ff00000)>>20) - 54;
            if (n< -50000) return tiny*x; 	/*underflow*/
	    }
        if (k==0x7ff) return x+x;		/* NaN or Inf */
        k = k+n;
        if (k >  0x7fe) return huge*SDL_NAME(copysign)(huge,x); /* overflow  */
        if (k > 0) 				/* normal result */
	    {SET_HIGH_WORD(x,(hx&0x800fffff)|(k<<20)); return x;}
        if (k <= -54) {
            if (n > 50000) 	/* in case integer overflow in n+k */
		return huge*SDL_NAME(copysign)(huge,x);	/*overflow*/
	    else return tiny*SDL_NAME(copysign)(tiny,x); 	/*underflow*/
	}
        k += 54;				/* subnormal result */
	SET_HIGH_WORD(x,(hx&0x800fffff)|(k<<20));
        return x*twom54;
}

#ifdef __STDC__
	double __ieee754_sqrt(double x)
#else
	double __ieee754_sqrt(x)
	double x;
#endif
{
	double z;
	int32_t sign = (int)0x80000000;
	int32_t ix0,s0,q,m,t,i;
	u_int32_t r,t1,s1,ix1,q1;

	EXTRACT_WORDS(ix0,ix1,x);

    /* take care of Inf and NaN */
	if((ix0&0x7ff00000)==0x7ff00000) {
	    return x*x+x;		/* sqrt(NaN)=NaN, sqrt(+inf)=+inf
					   sqrt(-inf)=sNaN */
	}
    /* take care of zero */
	if(ix0<=0) {
	    if(((ix0&(~sign))|ix1)==0) return x;/* sqrt(+-0) = +-0 */
	    else if(ix0<0)
		return (x-x)/(x-x);		/* sqrt(-ve) = sNaN */
	}
    /* normalize x */
	m = (ix0>>20);
	if(m==0) {				/* subnormal x */
	    while(ix0==0) {
		m -= 21;
		ix0 |= (ix1>>11); ix1 <<= 21;
	    }
	    for(i=0;(ix0&0x00100000)==0;i++) ix0<<=1;
	    m -= i-1;
	    ix0 |= (ix1>>(32-i));
	    ix1 <<= i;
	}
	m -= 1023;	/* unbias exponent */
	ix0 = (ix0&0x000fffff)|0x00100000;
	if(m&1){	/* odd m, double x to make it even */
	    ix0 += ix0 + ((ix1&sign)>>31);
	    ix1 += ix1;
	}
	m >>= 1;	/* m = [m/2] */

    /* generate sqrt(x) bit by bit */
	ix0 += ix0 + ((ix1&sign)>>31);
	ix1 += ix1;
	q = q1 = s0 = s1 = 0;	/* [q,q1] = sqrt(x) */
	r = 0x00200000;		/* r = moving bit from right to left */

	while(r!=0) {
	    t = s0+r;
	    if(t<=ix0) {
		s0   = t+r;
		ix0 -= t;
		q   += r;
	    }
	    ix0 += ix0 + ((ix1&sign)>>31);
	    ix1 += ix1;
	    r>>=1;
	}

	r = sign;
	while(r!=0) {
	    t1 = s1+r;
	    t  = s0;
	    if((t<ix0)||((t==ix0)&&(t1<=ix1))) {
		s1  = t1+r;
		if(((int32_t)(t1&sign)==sign)&&(s1&sign)==0) s0 += 1;
		ix0 -= t;
		if (ix1 < t1) ix0 -= 1;
		ix1 -= t1;
		q1  += r;
	    }
	    ix0 += ix0 + ((ix1&sign)>>31);
	    ix1 += ix1;
	    r>>=1;
	}

    /* use floating add to find out rounding direction */
	if((ix0|ix1)!=0) {
	    z = one-tiny; /* trigger inexact flag */
	    if (z>=one) {
	        z = one+tiny;
	        if (q1==(u_int32_t)0xffffffff) { q1=0; q += 1;}
		else if (z>one) {
		    if (q1==(u_int32_t)0xfffffffe) q+=1;
		    q1+=2;
		} else
	            q1 += (q1&1);
	    }
	}
	ix0 = (q>>1)+0x3fe00000;
	ix1 =  q1>>1;
	if ((q&1)==1) ix1 |= sign;
	ix0 += (m <<20);
	INSERT_WORDS(z,ix0,ix1);
	return z;
}


