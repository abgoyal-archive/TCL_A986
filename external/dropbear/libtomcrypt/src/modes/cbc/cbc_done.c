#include "tomcrypt.h"


#ifdef LTC_CBC_MODE

int cbc_done(symmetric_CBC *cbc)
{
   int err;
   LTC_ARGCHK(cbc != NULL);

   if ((err = cipher_is_valid(cbc->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[cbc->cipher].done(&cbc->key);
   return CRYPT_OK;
}

   

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/cbc/cbc_done.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:46:46 $ */
