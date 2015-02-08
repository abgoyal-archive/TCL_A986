#include <tommath.h>
#ifdef BN_MP_PRIME_FERMAT_C

int mp_prime_fermat (mp_int * a, mp_int * b, int *result)
{
  mp_int  t;
  int     err;

  /* default to composite  */
  *result = MP_NO;

  /* ensure b > 1 */
  if (mp_cmp_d(b, 1) != MP_GT) {
     return MP_VAL;
  }

  /* init t */
  if ((err = mp_init (&t)) != MP_OKAY) {
    return err;
  }

  /* compute t = b**a mod a */
  if ((err = mp_exptmod (b, a, a, &t)) != MP_OKAY) {
    goto LBL_T;
  }

  /* is it equal to b? */
  if (mp_cmp (&t, b) == MP_EQ) {
    *result = MP_YES;
  }

  err = MP_OKAY;
LBL_T:mp_clear (&t);
  return err;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_prime_fermat.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
