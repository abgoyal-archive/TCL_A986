#include "tomcrypt.h"


#ifdef LTC_CFB_MODE

int cfb_done(symmetric_CFB *cfb)
{
   int err;
   LTC_ARGCHK(cfb != NULL);

   if ((err = cipher_is_valid(cfb->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[cfb->cipher].done(&cfb->key);
   return CRYPT_OK;
}

   

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/cfb/cfb_done.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
