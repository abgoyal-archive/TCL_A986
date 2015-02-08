#include "tomcrypt.h"



#ifdef LTC_CBC_MODE

int cbc_setiv(const unsigned char *IV, unsigned long len, symmetric_CBC *cbc)
{
   LTC_ARGCHK(IV  != NULL);
   LTC_ARGCHK(cbc != NULL);
   if (len != (unsigned long)cbc->blocklen) {
      return CRYPT_INVALID_ARG;
   }
   XMEMCPY(cbc->IV, IV, len);
   return CRYPT_OK;
}

#endif 


/* $Source: /cvs/libtom/libtomcrypt/src/modes/cbc/cbc_setiv.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/06/29 01:46:46 $ */
