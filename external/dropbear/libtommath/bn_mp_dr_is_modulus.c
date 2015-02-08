#include <tommath.h>
#ifdef BN_MP_DR_IS_MODULUS_C

/* determines if a number is a valid DR modulus */
int mp_dr_is_modulus(mp_int *a)
{
   int ix;

   /* must be at least two digits */
   if (a->used < 2) {
      return 0;
   }

   /* must be of the form b**k - a [a <= b] so all
    * but the first digit must be equal to -1 (mod b).
    */
   for (ix = 1; ix < a->used; ix++) {
       if (a->dp[ix] != MP_MASK) {
          return 0;
       }
   }
   return 1;
}

#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_dr_is_modulus.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
