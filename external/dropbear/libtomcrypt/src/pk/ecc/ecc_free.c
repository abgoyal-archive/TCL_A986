
#include "tomcrypt.h"


#ifdef MECC

void ecc_free(ecc_key *key)
{
   LTC_ARGCHKVD(key != NULL);
   mp_clear_multi(key->pubkey.x, key->pubkey.y, key->pubkey.z, key->k, NULL);
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_free.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/06/09 01:38:14 $ */

