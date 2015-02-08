#include "tomcrypt.h"


#ifdef LTC_CFB_MODE

int cfb_getiv(unsigned char *IV, unsigned long *len, symmetric_CFB *cfb)
{
   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(len != NULL);
   LTC_ARGCHK(cfb != NULL);
   if ((unsigned long)cfb->blocklen > *len) {
      *len = cfb->blocklen;
      return CRYPT_BUFFER_OVERFLOW;
   }
   XMEMCPY(IV, cfb->IV, cfb->blocklen);
   *len = cfb->blocklen;

   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/cfb/cfb_getiv.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:51:34 $ */
