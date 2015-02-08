#include <tommath.h>
#ifdef BN_MP_CMP_C

/* compare two ints (signed)*/
int
mp_cmp (mp_int * a, mp_int * b)
{
  /* compare based on sign */
  if (a->sign != b->sign) {
     if (a->sign == MP_NEG) {
        return MP_LT;
     } else {
        return MP_GT;
     }
  }
  
  /* compare digits */
  if (a->sign == MP_NEG) {
     /* if negative compare opposite direction */
     return mp_cmp_mag(b, a);
  } else {
     return mp_cmp_mag(a, b);
  }
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_cmp.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
