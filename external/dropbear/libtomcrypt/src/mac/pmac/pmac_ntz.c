#include "tomcrypt.h"


#ifdef LTC_PMAC

int pmac_ntz(unsigned long x)
{
   int c;
   x &= 0xFFFFFFFFUL;
   c = 0;
   while ((x & 1) == 0) {
      ++c;
      x >>= 1;
   }
   return c;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/mac/pmac/pmac_ntz.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/11/03 00:39:49 $ */
