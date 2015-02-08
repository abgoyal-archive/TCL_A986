#include <tommath.h>
#ifdef BN_MP_INVMOD_C

/* hac 14.61, pp608 */
int mp_invmod (mp_int * a, mp_int * b, mp_int * c)
{
  /* b cannot be negative */
  if (b->sign == MP_NEG || mp_iszero(b) == 1) {
    return MP_VAL;
  }

#ifdef BN_FAST_MP_INVMOD_C
  /* if the modulus is odd we can use a faster routine instead */
  if (mp_isodd (b) == 1) {
    return fast_mp_invmod (a, b, c);
  }
#endif

#ifdef BN_MP_INVMOD_SLOW_C
  return mp_invmod_slow(a, b, c);
#endif

  return MP_VAL;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_invmod.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
