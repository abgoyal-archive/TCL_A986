#include "tomcrypt.h"


int register_cipher(const struct ltc_cipher_descriptor *cipher)
{
   int x;

   LTC_ARGCHK(cipher != NULL);

   /* is it already registered? */
   LTC_MUTEX_LOCK(&ltc_cipher_mutex);
   for (x = 0; x < TAB_SIZE; x++) {
       if (cipher_descriptor[x].name != NULL && cipher_descriptor[x].ID == cipher->ID) {
          LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
          return x;
       }
   }

   /* find a blank spot */
   for (x = 0; x < TAB_SIZE; x++) {
       if (cipher_descriptor[x].name == NULL) {
          XMEMCPY(&cipher_descriptor[x], cipher, sizeof(struct ltc_cipher_descriptor));
          LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
          return x;
       }
   }

   /* no spot */
   LTC_MUTEX_UNLOCK(&ltc_cipher_mutex);
   return -1;
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_register_cipher.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
