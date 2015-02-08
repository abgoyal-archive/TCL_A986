#include <tommath.h>
#ifdef BN_MP_ABS_C

int
mp_abs (mp_int * a, mp_int * b)
{
  int     res;

  /* copy a to b */
  if (a != b) {
     if ((res = mp_copy (a, b)) != MP_OKAY) {
       return res;
     }
  }

  /* force the sign of b to positive */
  b->sign = MP_ZPOS;

  return MP_OKAY;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_abs.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
