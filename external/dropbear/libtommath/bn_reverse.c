#include <tommath.h>
#ifdef BN_REVERSE_C

/* reverse an array, used for radix code */
void
bn_reverse (unsigned char *s, int len)
{
  int     ix, iy;
  unsigned char t;

  ix = 0;
  iy = len - 1;
  while (ix < iy) {
    t     = s[ix];
    s[ix] = s[iy];
    s[iy] = t;
    ++ix;
    --iy;
  }
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_reverse.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
