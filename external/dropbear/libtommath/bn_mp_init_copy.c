#include <tommath.h>
#ifdef BN_MP_INIT_COPY_C

/* creates "a" then copies b into it */
int mp_init_copy (mp_int * a, mp_int * b)
{
  int     res;

  if ((res = mp_init (a)) != MP_OKAY) {
    return res;
  }
  return mp_copy (b, a);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_init_copy.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
