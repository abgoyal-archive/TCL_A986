#include <tommath.h>
#ifdef BN_MP_CLAMP_C

void
mp_clamp (mp_int * a)
{
  /* decrease used while the most significant digit is
   * zero.
   */
  while (a->used > 0 && a->dp[a->used - 1] == 0) {
    --(a->used);
  }

  /* reset the sign flag if used == 0 */
  if (a->used == 0) {
    a->sign = MP_ZPOS;
  }
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_clamp.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
