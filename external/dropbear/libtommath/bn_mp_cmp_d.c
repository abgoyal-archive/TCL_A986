#include <tommath.h>
#ifdef BN_MP_CMP_D_C

/* compare a digit */
int mp_cmp_d(mp_int * a, mp_digit b)
{
  /* compare based on sign */
  if (a->sign == MP_NEG) {
    return MP_LT;
  }

  /* compare based on magnitude */
  if (a->used > 1) {
    return MP_GT;
  }

  /* compare the only digit of a to b */
  if (a->dp[0] > b) {
    return MP_GT;
  } else if (a->dp[0] < b) {
    return MP_LT;
  } else {
    return MP_EQ;
  }
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_cmp_d.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
