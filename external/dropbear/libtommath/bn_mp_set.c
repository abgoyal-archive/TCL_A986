#include <tommath.h>
#ifdef BN_MP_SET_C

/* set to a digit */
void mp_set (mp_int * a, mp_digit b)
{
  mp_zero (a);
  a->dp[0] = b & MP_MASK;
  a->used  = (a->dp[0] != 0) ? 1 : 0;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_set.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
