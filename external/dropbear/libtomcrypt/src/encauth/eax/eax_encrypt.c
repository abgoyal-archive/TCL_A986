
#include "tomcrypt.h"

#ifdef EAX_MODE

int eax_encrypt(eax_state *eax, const unsigned char *pt, unsigned char *ct, 
                unsigned long length)
{
   int err;
   
   LTC_ARGCHK(eax != NULL);
   LTC_ARGCHK(pt  != NULL);
   LTC_ARGCHK(ct  != NULL);

   /* encrypt */
   if ((err = ctr_encrypt(pt, ct, length, &eax->ctr)) != CRYPT_OK) {
      return err;
   }

   /* omac ciphertext */
   return omac_process(&eax->ctomac, ct, length);
}

#endif


/* $Source: /cvs/libtom/libtomcrypt/src/encauth/eax/eax_encrypt.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
