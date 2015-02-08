#include <tommath.h>
#ifdef BN_MP_EXCH_C

void
mp_exch (mp_int * a, mp_int * b)
{
  mp_int  t;

  t  = *a;
  *a = *b;
  *b = t;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_exch.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
