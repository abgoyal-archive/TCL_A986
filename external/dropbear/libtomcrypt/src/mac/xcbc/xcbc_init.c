#include "tomcrypt.h"


#ifdef LTC_XCBC

int xcbc_init(xcbc_state *xcbc, int cipher, const unsigned char *key, unsigned long keylen)
{
   int            x, y, err;
   symmetric_key *skey;

   LTC_ARGCHK(xcbc != NULL);
   LTC_ARGCHK(key  != NULL);

   /* schedule the key */
   if ((err = cipher_is_valid(cipher)) != CRYPT_OK) {
      return err;
   }

#ifdef LTC_FAST
   if (cipher_descriptor[cipher].block_length % sizeof(LTC_FAST_TYPE)) {
       return CRYPT_INVALID_ARG;
   }
#endif

   /* schedule the user key */
   skey = XCALLOC(1, sizeof(*skey));
   if (skey == NULL) {
      return CRYPT_MEM;
   }

   if ((err = cipher_descriptor[cipher].setup(key, keylen, 0, skey)) != CRYPT_OK) {
      goto done;
   }
   
   /* make the three keys */
   for (y = 0; y < 3; y++) {
     for (x = 0; x < cipher_descriptor[cipher].block_length; x++) {
        xcbc->K[y][x] = y + 1;
     }
     cipher_descriptor[cipher].ecb_encrypt(xcbc->K[y], xcbc->K[y], skey);
   }

   /* setup K1 */
   err = cipher_descriptor[cipher].setup(xcbc->K[0], cipher_descriptor[cipher].block_length, 0, &xcbc->key);
 
   /* setup struct */
   zeromem(xcbc->IV, cipher_descriptor[cipher].block_length);
   xcbc->blocksize = cipher_descriptor[cipher].block_length;
   xcbc->cipher    = cipher;
   xcbc->buflen    = 0;
done:
   cipher_descriptor[cipher].done(skey);
#ifdef LTC_CLEAN_STACK
   zeromem(skey, sizeof(*skey));
#endif
   XFREE(skey);
   return err;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/mac/xcbc/xcbc_init.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/11/07 03:23:46 $ */

