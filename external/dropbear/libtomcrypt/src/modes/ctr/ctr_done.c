#include "tomcrypt.h"


#ifdef LTC_CTR_MODE

int ctr_done(symmetric_CTR *ctr)
{
   int err;
   LTC_ARGCHK(ctr != NULL);

   if ((err = cipher_is_valid(ctr->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[ctr->cipher].done(&ctr->key);
   return CRYPT_OK;
}

   

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/ctr/ctr_done.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:46:46 $ */
