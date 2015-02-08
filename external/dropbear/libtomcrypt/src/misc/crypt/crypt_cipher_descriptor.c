#include "tomcrypt.h"


struct ltc_cipher_descriptor cipher_descriptor[TAB_SIZE] = {
{ NULL, 0, 0, 0, 0, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL }
 };

LTC_MUTEX_GLOBAL(ltc_cipher_mutex)


/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_cipher_descriptor.c,v $ */
/* $Revision: 1.12 $ */
/* $Date: 2006/11/08 23:01:06 $ */
