#include "tomcrypt.h"


#ifdef MDSA

void dsa_free(dsa_key *key)
{
   LTC_ARGCHKVD(key != NULL);
   mp_clear_multi(key->g, key->q, key->p, key->x, key->y, NULL);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/dsa/dsa_free.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/09 01:38:13 $ */
