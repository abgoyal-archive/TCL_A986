#include "tomcrypt.h"



#ifdef LTC_CFB_MODE

int cfb_start(int cipher, const unsigned char *IV, const unsigned char *key, 
              int keylen, int num_rounds, symmetric_CFB *cfb)
{
   int x, err;

   LTC_ARGCHK(IV != NULL);
   LTC_ARGCHK(key != NULL);
   LTC_ARGCHK(cfb != NULL);

   if ((err = cipher_is_valid(cipher)) != CRYPT_OK) {
      return err;
   }
   

   /* copy data */
   cfb->cipher = cipher;
   cfb->blocklen = cipher_descriptor[cipher].block_length;
   for (x = 0; x < cfb->blocklen; x++)
       cfb->IV[x] = IV[x];

   /* init the cipher */
   if ((err = cipher_descriptor[cipher].setup(key, keylen, num_rounds, &cfb->key)) != CRYPT_OK) {
      return err;
   }

   /* encrypt the IV */
   cfb->padlen = 0;
   return cipher_descriptor[cfb->cipher].ecb_encrypt(cfb->IV, cfb->IV, &cfb->key);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/cfb/cfb_start.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
