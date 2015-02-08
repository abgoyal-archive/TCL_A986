#include "tomcrypt.h"


#ifdef LTC_CFB_MODE

int cfb_setiv(const unsigned char *IV, unsigned long len, symmetric_CFB *cfb)
{
   int err;
   
   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(cfb != NULL);

   if ((err = cipher_is_valid(cfb->cipher)) != CRYPT_OK) {
       return err;
   }
   
   if (len != (unsigned long)cfb->blocklen) {
      return CRYPT_INVALID_ARG;
   }
      
   /* force next block */
   cfb->padlen = 0;
   return cipher_descriptor[cfb->cipher].ecb_encrypt(IV, cfb->IV, &cfb->key);
}

#endif 


/* $Source: /cvs/libtom/libtomcrypt/src/modes/cfb/cfb_setiv.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
