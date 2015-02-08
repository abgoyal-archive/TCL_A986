#include "tomcrypt.h"

struct ltc_prng_descriptor prng_descriptor[TAB_SIZE] = {
{ NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL }
};

LTC_MUTEX_GLOBAL(ltc_prng_mutex)


/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_prng_descriptor.c,v $ */
/* $Revision: 1.7 $ */
/* $Date: 2006/03/31 14:15:35 $ */
