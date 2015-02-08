#include "tomcrypt.h"


int find_cipher(const char *name)
{
   int x;
   LTC_ARGCHK(name != NULL);
   LTC_MUTEX_LOCK(&ltc_cipher_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
       if (cipher_descriptor[x].name != NULL && !XSTRCMP(cipher_descriptor[x].name, name)) {
          LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
          return x;
       }
   }
   LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
   return -1;
}


/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_find_cipher.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/11/29 23:43:57 $ */
