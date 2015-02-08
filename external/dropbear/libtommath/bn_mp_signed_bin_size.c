#include <tommath.h>
#ifdef BN_MP_SIGNED_BIN_SIZE_C

/* get the size for an signed equivalent */
int mp_signed_bin_size (mp_int * a)
{
  return 1 + mp_unsigned_bin_size (a);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_signed_bin_size.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
