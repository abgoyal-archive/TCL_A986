#include "tomcrypt.h"


int register_hash(const struct ltc_hash_descriptor *hash)
{
   int x;

   LTC_ARGCHK(hash != NULL);

   /* is it already registered? */
   LTC_MUTEX_LOCK(&ltc_hash_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
       if (XMEMCMP(&hash_descriptor[x], hash, sizeof(struct ltc_hash_descriptor)) == 0) {
          LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
          return x;
       }
   }

   /* find a blank spot */
   for (x = 0; x < TAB_SIZE; x++) {
       if (hash_descriptor[x].name == NULL) {
          XMEMCPY(&hash_descriptor[x], hash, sizeof(struct ltc_hash_descriptor));
          LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
          return x;
       }
   }

   /* no spot */
   LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
   return -1;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_register_hash.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/11/01 09:28:17 $ */
