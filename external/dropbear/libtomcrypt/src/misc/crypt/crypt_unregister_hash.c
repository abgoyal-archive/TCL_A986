#include "tomcrypt.h"


int unregister_hash(const struct ltc_hash_descriptor *hash)
{
   int x;

   LTC_ARGCHK(hash != NULL);

   /* is it already registered? */
   LTC_MUTEX_LOCK(&ltc_hash_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
       if (XMEMCMP(&hash_descriptor[x], hash, sizeof(struct ltc_hash_descriptor)) == 0) {
          hash_descriptor[x].name = NULL;
          LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
          return CRYPT_OK;
       }
   }
   LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
   return CRYPT_ERROR;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_unregister_hash.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/11/01 09:28:17 $ */
