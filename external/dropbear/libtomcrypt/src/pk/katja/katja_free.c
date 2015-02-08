#include "tomcrypt.h"


#ifdef MKAT

void katja_free(katja_key *key)
{
   LTC_ARGCHK(key != NULL);
   mp_clear_multi( key->d,  key->N,  key->dQ,  key->dP,
                   key->qP,  key->p,  key->q, key->pq, NULL);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/katja/katja_free.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/03/31 14:15:35 $ */
