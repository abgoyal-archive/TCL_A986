#include "tomcrypt.h"


#ifdef LTC_CTR_MODE

int ctr_getiv(unsigned char *IV, unsigned long *len, symmetric_CTR *ctr)
{
   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(len != NULL);
   LTC_ARGCHK(ctr != NULL);
   if ((unsigned long)ctr->blocklen > *len) {
      *len = ctr->blocklen;
      return CRYPT_BUFFER_OVERFLOW;
   }
   XMEMCPY(IV, ctr->ctr, ctr->blocklen);
   *len = ctr->blocklen;

   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/modes/ctr/ctr_getiv.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/29 01:46:46 $ */
