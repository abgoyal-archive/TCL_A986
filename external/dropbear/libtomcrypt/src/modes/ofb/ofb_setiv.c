#include "tomcrypt.h"


#ifdef LTC_OFB_MODE

int ofb_setiv(const unsigned char *IV, unsigned long len, symmetric_OFB *ofb)
{
   int err;

   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(ofb != NULL);

   if ((err = cipher_is_valid(ofb->cipher)) != CRYPT_OK) {
       return err;
   }

   if (len != (unsigned long)ofb->blocklen) {
      return CRYPT_INVALID_ARG;
   }

   /* force next block */
   ofb->padlen = 0;
   return cipher_descriptor[ofb->cipher].ecb_encrypt(IV, ofb->IV, &ofb->key);
}

#endif 


/* $Source: /cvs/libtom/libtomcrypt/src/modes/ofb/ofb_setiv.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
