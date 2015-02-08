#include <tommath.h>
#ifdef BN_MP_TO_UNSIGNED_BIN_N_C

/* store in unsigned [big endian] format */
int mp_to_unsigned_bin_n (mp_int * a, unsigned char *b, unsigned long *outlen)
{
   if (*outlen < (unsigned long)mp_unsigned_bin_size(a)) {
      return MP_VAL;
   }
   *outlen = mp_unsigned_bin_size(a);
   return mp_to_unsigned_bin(a, b);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_to_unsigned_bin_n.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
