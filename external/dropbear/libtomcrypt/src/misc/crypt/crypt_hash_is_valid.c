#include "tomcrypt.h"


int hash_is_valid(int idx)
{
   LTC_MUTEX_LOCK(&ltc_hash_mutex);
   if (idx < 0 || idx >= TAB_SIZE || hash_descriptor[idx].name == NULL) {
      LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
      return CRYPT_INVALID_HASH;
   }
   LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
   return CRYPT_OK;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_hash_is_valid.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
