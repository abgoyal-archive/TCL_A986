#include "tomcrypt.h"


int find_hash_id(unsigned char ID)
{
   int x;
   LTC_MUTEX_LOCK(&ltc_hash_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
      if (hash_descriptor[x].ID == ID) {
          x = (hash_descriptor[x].name == NULL) ? -1 : x;
          LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
          return x;
      }
   }
   LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
   return -1;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_find_hash_id.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/03/31 14:15:35 $ */
