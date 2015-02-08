#include "tomcrypt.h"


int cipher_is_valid(int idx)
{
   LTC_MUTEX_LOCK(&ltc_cipher_mutex);
   if (idx < 0 || idx >= TAB_SIZE || cipher_descriptor[idx].name == NULL) {
      LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
      return CRYPT_INVALID_CIPHER;
   }
   LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
   return CRYPT_OK;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_cipher_is_valid.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
