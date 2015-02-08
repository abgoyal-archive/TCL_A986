#include "tomcrypt.h"

  
#ifdef LTC_CTR_MODE

int ctr_setiv(const unsigned char *IV, unsigned long len, symmetric_CTR *ctr)
{
   int err;
   
   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(ctr != NULL);

   /* bad param? */
   if ((err = cipher_is_valid(ctr->cipher)) != CRYPT_OK) {
      return err;
   }
   
   if (len != (unsigned long)ctr->blocklen) {
      return CRYPT_INVALID_ARG;
   }

   /* set IV */
   XMEMCPY(ctr->ctr, IV, len);
   
   /* force next block */
   ctr->padlen = 0;
   return cipher_descriptor[ctr->cipher].ecb_encrypt(IV, ctr->pad, &ctr->key);
}

#endif 


/* $Source: /cvs/libtom/libtomcrypt/src/modes/ctr/ctr_setiv.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:46:46 $ */
