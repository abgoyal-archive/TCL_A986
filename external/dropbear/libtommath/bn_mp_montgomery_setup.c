#include <tommath.h>
#ifdef BN_MP_MONTGOMERY_SETUP_C

/* setups the montgomery reduction stuff */
int
mp_montgomery_setup (mp_int * n, mp_digit * rho)
{
  mp_digit x, b;

  b = n->dp[0];

  if ((b & 1) == 0) {
    return MP_VAL;
  }

  x = (((b + 2) & 4) << 1) + b; /* here x*a==1 mod 2**4 */
  x *= 2 - b * x;               /* here x*a==1 mod 2**8 */
#if !defined(MP_8BIT)
  x *= 2 - b * x;               /* here x*a==1 mod 2**16 */
#endif
#if defined(MP_64BIT) || !(defined(MP_8BIT) || defined(MP_16BIT))
  x *= 2 - b * x;               /* here x*a==1 mod 2**32 */
#endif
#ifdef MP_64BIT
  x *= 2 - b * x;               /* here x*a==1 mod 2**64 */
#endif

  /* rho = -1/m mod b */
  *rho = (unsigned long)(((mp_word)1 << ((mp_word) DIGIT_BIT)) - x) & MP_MASK;

  return MP_OKAY;
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_montgomery_setup.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/12/04 21:34:03 $ */
