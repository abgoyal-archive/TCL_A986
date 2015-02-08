
#include "tomcrypt.h"

#ifdef EAX_MODE

int eax_decrypt(eax_state *eax, const unsigned char *ct, unsigned char *pt, 
                unsigned long length)
{
   int err;
   
   LTC_ARGCHK(eax != NULL);
   LTC_ARGCHK(pt  != NULL);
   LTC_ARGCHK(ct  != NULL);

   /* omac ciphertext */
   if ((err = omac_process(&eax->ctomac, ct, length)) != CRYPT_OK) {
      return err;
   }

   /* decrypt  */
   return ctr_decrypt(ct, pt, length, &eax->ctr);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/eax/eax_decrypt.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
