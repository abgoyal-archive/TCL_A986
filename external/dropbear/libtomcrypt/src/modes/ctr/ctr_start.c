#include "tomcrypt.h"



#ifdef LTC_CTR_MODE

int ctr_start(               int   cipher, 
              const unsigned char *IV, 
              const unsigned char *key,       int keylen, 
                             int  num_rounds, int ctr_mode,
                   symmetric_CTR *ctr)
{
   int x, err;

   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(key != NULL);
   LTC_ARGCHK(ctr != NULL);

   /* bad param? */
   if ((err = cipher_is_valid(cipher)) != CRYPT_OK) {
      return err;
   }

   /* setup cipher */
   if ((err = cipher_descriptor[cipher].setup(key, keylen, num_rounds, &ctr->key)) != CRYPT_OK) {
      return err;
   }

   /* copy ctr */
   ctr->blocklen = cipher_descriptor[cipher].block_length;
   ctr->cipher   = cipher;
   ctr->padlen   = 0;
   ctr->mode     = ctr_mode & 1;
   for (x = 0; x < ctr->blocklen; x++) {
       ctr->ctr[x] = IV[x];
   }

   if (ctr_mode & LTC_CTR_RFC3686) {
      /* increment the IV as per RFC 3686 */
      if (ctr->mode == CTR_COUNTER_LITTLE_ENDIAN) {
         /* little-endian */
         for (x = 0; x < ctr->blocklen; x++) {
             ctr->ctr[x] = (ctr->ctr[x] + (unsigned char)1) & (unsigned char)255;
             if (ctr->ctr[x] != (unsigned char)0) {
                break;
             }
         }
      } else {
         /* big-endian */
         for (x = ctr->blocklen-1; x >= 0; x--) {
             ctr->ctr[x] = (ctr->ctr[x] + (unsigned char)1) & (unsigned char)255;
             if (ctr->ctr[x] != (unsigned char)0) {
                break;
             }
         }
      }
   }

   return cipher_descriptor[ctr->cipher].ecb_encrypt(ctr->ctr, ctr->pad, &ctr->key); 
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/ctr/ctr_start.c,v $ */
/* $Revision: 1.11 $ */
/* $Date: 2006/11/05 01:46:35 $ */
