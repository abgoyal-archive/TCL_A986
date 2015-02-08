
#include "tomcrypt.h"


#ifdef MECC

int ltc_ecc_is_valid_idx(int n)
{
   int x;

   for (x = 0; ltc_ecc_sets[x].size != 0; x++);
   /* -1 is a valid index --- indicating that the domain params were supplied by the user */
   if ((n >= -1) || (n < x)) {
      return 1;
   }
   return 0;
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ltc_ecc_is_valid_idx.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/11/21 00:10:18 $ */

