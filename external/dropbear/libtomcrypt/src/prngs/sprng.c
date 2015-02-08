#include "tomcrypt.h"

   

#ifdef SPRNG

const struct ltc_prng_descriptor sprng_desc =
{
    "sprng", 0,
    &sprng_start,
    &sprng_add_entropy,
    &sprng_ready,
    &sprng_read,
    &sprng_done,
    &sprng_export,
    &sprng_import,
    &sprng_test
};

int sprng_start(prng_state *prng)
{
   return CRYPT_OK;  
}

int sprng_add_entropy(const unsigned char *in, unsigned long inlen, prng_state *prng)
{
   return CRYPT_OK;
}

int sprng_ready(prng_state *prng)
{
   return CRYPT_OK;
}

unsigned long sprng_read(unsigned char *out, unsigned long outlen, prng_state *prng)
{
   LTC_ARGCHK(out != NULL);
   return rng_get_bytes(out, outlen, NULL);
}

int sprng_done(prng_state *prng)
{
   return CRYPT_OK;
}

int sprng_export(unsigned char *out, unsigned long *outlen, prng_state *prng)
{
   LTC_ARGCHK(outlen != NULL);

   *outlen = 0;
   return CRYPT_OK;
}
 
int sprng_import(const unsigned char *in, unsigned long inlen, prng_state *prng)
{
   return CRYPT_OK;
}

int sprng_test(void)
{
   return CRYPT_OK;
}

#endif


 

/* $Source: /cvs/libtom/libtomcrypt/src/prngs/sprng.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
