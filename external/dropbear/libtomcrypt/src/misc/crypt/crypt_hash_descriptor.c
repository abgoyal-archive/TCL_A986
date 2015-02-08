#include "tomcrypt.h"


struct ltc_hash_descriptor hash_descriptor[TAB_SIZE] = {
{ NULL, 0, 0, 0, { 0 }, 0, NULL, NULL, NULL, NULL, NULL }
};

LTC_MUTEX_GLOBAL(ltc_hash_mutex)


/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_hash_descriptor.c,v $ */
/* $Revision: 1.9 $ */
/* $Date: 2006/03/31 14:15:35 $ */
