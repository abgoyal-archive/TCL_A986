#include "tomcrypt.h"


#ifdef LTC_LRW_MODE

int lrw_done(symmetric_LRW *lrw) 
{
   int err;

   LTC_ARGCHK(lrw != NULL);
 
   if ((err = cipher_is_valid(lrw->cipher)) != CRYPT_OK) {
      return err;
   }
   cipher_descriptor[lrw->cipher].done(&lrw->key);

   return CRYPT_OK;
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/modes/lrw/lrw_done.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:53:13 $ */
