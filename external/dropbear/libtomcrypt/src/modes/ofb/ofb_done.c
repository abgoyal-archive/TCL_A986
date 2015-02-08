#include "tomcrypt.h"


#ifdef LTC_OFB_MODE

int ofb_done(symmetric_OFB *ofb)
{
   int err;
   LTC_ARGCHK(ofb != NULL);

   if ((err = cipher_is_valid(ofb->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[ofb->cipher].done(&ofb->key);
   return CRYPT_OK;
}

   

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/ofb/ofb_done.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
