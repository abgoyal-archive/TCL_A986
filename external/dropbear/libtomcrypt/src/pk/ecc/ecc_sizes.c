
#include "tomcrypt.h"


#ifdef MECC

void ecc_sizes(int *low, int *high)
{
 int i;
 LTC_ARGCHKVD(low  != NULL);
 LTC_ARGCHKVD(high != NULL);

 *low = INT_MAX;
 *high = 0;
 for (i = 0; ltc_ecc_sets[i].size != 0; i++) {
     if (ltc_ecc_sets[i].size < *low)  {
        *low  = ltc_ecc_sets[i].size;
     }
     if (ltc_ecc_sets[i].size > *high) {
        *high = ltc_ecc_sets[i].size;
     }
 }
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_sizes.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/06/09 01:38:14 $ */

