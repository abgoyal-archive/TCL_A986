#include <tommath.h>
#ifdef BN_MP_MOD_D_C

int
mp_mod_d (mp_int * a, mp_digit b, mp_digit * c)
{
  return mp_div_d(a, b, NULL, c);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_mod_d.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
