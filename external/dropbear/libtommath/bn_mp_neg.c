#include <tommath.h>
#ifdef BN_MP_NEG_C

/* b = -a */
int mp_neg (mp_int * a, mp_int * b)
{
  int     res;
  if (a != b) {
     if ((res = mp_copy (a, b)) != MP_OKAY) {
        return res;
     }
  }

  if (mp_iszero(b) != MP_YES) {
     b->sign = (a->sign == MP_ZPOS) ? MP_NEG : MP_ZPOS;
  } else {
     b->sign = MP_ZPOS;
  }

  return MP_OKAY;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_neg.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
