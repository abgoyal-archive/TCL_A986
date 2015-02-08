#include "tomcrypt.h"


int prng_is_valid(int idx)
{
   LTC_MUTEX_LOCK(&ltc_prng_mutex);
   if (idx < 0 || idx >= TAB_SIZE || prng_descriptor[idx].name == NULL) {
      LTC_MUTEX_UNLOCK(&ltc_prng_mutex);
      return CRYPT_INVALID_PRNG;
   }
   LTC_MUTEX_UNLOCK(&ltc_prng_mutex);
   return CRYPT_OK;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_prng_is_valid.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
