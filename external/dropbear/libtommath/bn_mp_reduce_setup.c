#include <tommath.h>
#ifdef BN_MP_REDUCE_SETUP_C

int mp_reduce_setup (mp_int * a, mp_int * b)
{
  int     res;
  
  if ((res = mp_2expt (a, b->used * 2 * DIGIT_BIT)) != MP_OKAY) {
    return res;
  }
  return mp_div (a, b, a, NULL);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_reduce_setup.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
