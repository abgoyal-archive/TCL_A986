
#include "tomcrypt.h"


#ifdef MECC

int ecc_get_size(ecc_key *key)
{
   LTC_ARGCHK(key != NULL);
   if (ltc_ecc_is_valid_idx(key->idx))
      return key->dp->size;
   else
      return INT_MAX; /* large value known to cause it to fail when passed to ecc_make_key() */
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_get_size.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/11/21 00:10:18 $ */

