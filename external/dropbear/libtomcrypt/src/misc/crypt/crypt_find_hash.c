#include "tomcrypt.h"


int find_hash(const char *name)
{
   int x;
   LTC_ARGCHK(name != NULL);
   LTC_MUTEX_LOCK(&ltc_hash_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
       if (hash_descriptor[x].name != NULL && XSTRCMP(hash_descriptor[x].name, name) == 0) {
          LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
          return x;
       }
   }
   LTC_MUTEX_UNLOCK(&ltc_hash_mutex);
   return -1;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_find_hash.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/11/29 23:43:57 $ */
