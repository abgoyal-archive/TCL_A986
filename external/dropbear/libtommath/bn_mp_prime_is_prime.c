#include <tommath.h>
#ifdef BN_MP_PRIME_IS_PRIME_C

int mp_prime_is_prime (mp_int * a, int t, int *result)
{
  mp_int  b;
  int     ix, err, res;

  /* default to no */
  *result = MP_NO;

  /* valid value of t? */
  if (t <= 0 || t > PRIME_SIZE) {
    return MP_VAL;
  }

  /* is the input equal to one of the primes in the table? */
  for (ix = 0; ix < PRIME_SIZE; ix++) {
      if (mp_cmp_d(a, ltm_prime_tab[ix]) == MP_EQ) {
         *result = 1;
         return MP_OKAY;
      }
  }

  /* first perform trial division */
  if ((err = mp_prime_is_divisible (a, &res)) != MP_OKAY) {
    return err;
  }

  /* return if it was trivially divisible */
  if (res == MP_YES) {
    return MP_OKAY;
  }

  /* now perform the miller-rabin rounds */
  if ((err = mp_init (&b)) != MP_OKAY) {
    return err;
  }

  for (ix = 0; ix < t; ix++) {
    /* set the prime */
    mp_set (&b, ltm_prime_tab[ix]);

    if ((err = mp_prime_miller_rabin (a, &b, &res)) != MP_OKAY) {
      goto LBL_B;
    }

    if (res == MP_NO) {
      goto LBL_B;
    }
  }

  /* passed the test */
  *result = MP_YES;
LBL_B:mp_clear (&b);
  return err;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_prime_is_prime.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
