#include "tomcrypt.h"


#ifdef MRSA

void rsa_free(rsa_key *key)
{
   LTC_ARGCHKVD(key != NULL);
   mp_clear_multi(key->e, key->d, key->N, key->dQ, key->dP, key->qP, key->p, key->q, NULL);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/rsa/rsa_free.c,v $ */
/* $Revision: 1.8 $ */
/* $Date: 2006/12/04 22:23:27 $ */
