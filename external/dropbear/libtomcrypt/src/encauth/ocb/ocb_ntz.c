

#include "tomcrypt.h"

#ifdef OCB_MODE

int ocb_ntz(unsigned long x)
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

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/ocb/ocb_ntz.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
