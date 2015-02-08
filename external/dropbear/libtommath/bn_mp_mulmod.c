#include <tommath.h>
#ifdef BN_MP_MULMOD_C

/* d = a * b (mod c) */
int mp_mulmod (mp_int * a, mp_int * b, mp_int * c, mp_int * d)
{
  int     res;
  mp_int  t;

  if ((res = mp_init (&t)) != MP_OKAY) {
    return res;
  }

  if ((res = mp_mul (a, b, &t)) != MP_OKAY) {
    mp_clear (&t);
    return res;
  }
  res = mp_mod (&t, c, d);
  mp_clear (&t);
  return res;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_mulmod.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:18:44 $ */
