#include <tommath.h>
#ifdef BN_MP_2EXPT_C

int
mp_2expt (mp_int * a, int b)
{
  int     res;

  /* zero a as per default */
  mp_zero (a);

  /* grow a to accomodate the single bit */
  if ((res = mp_grow (a, b / DIGIT_BIT + 1)) != MP_OKAY) {
    return res;
  }

  /* set the used count of where the bit will go */
  a->used = b / DIGIT_BIT + 1;

  /* put the single bit in its place */
  a->dp[b / DIGIT_BIT] = ((mp_digit)1) << (b % DIGIT_BIT);

  return MP_OKAY;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_2expt.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
