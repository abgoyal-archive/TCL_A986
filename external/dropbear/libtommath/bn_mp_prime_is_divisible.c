#include <tommath.h>
#ifdef BN_MP_PRIME_IS_DIVISIBLE_C

int mp_prime_is_divisible (mp_int * a, int *result)
{
  int     err, ix;
  mp_digit res;

  /* default to not */
  *result = MP_NO;

  for (ix = 0; ix < PRIME_SIZE; ix++) {
    /* what is a mod LBL_prime_tab[ix] */
    if ((err = mp_mod_d (a, ltm_prime_tab[ix], &res)) != MP_OKAY) {
      return err;
    }

    /* is the residue zero? */
    if (res == 0) {
      *result = MP_YES;
      return MP_OKAY;
    }
  }

  return MP_OKAY;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_prime_is_divisible.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
