#include "tomcrypt.h"


#ifdef LTC_F8_MODE

int f8_done(symmetric_F8 *f8)
{
   int err;
   LTC_ARGCHK(f8 != NULL);

   if ((err = cipher_is_valid(f8->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[f8->cipher].done(&f8->key);
   return CRYPT_OK;
}

   

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/f8/f8_done.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/06/16 22:49:25 $ */
