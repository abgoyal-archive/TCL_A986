#include "tomcrypt.h"


#ifdef LTC_F8_MODE

int f8_decrypt(const unsigned char *ct, unsigned char *pt, unsigned long len, symmetric_F8 *f8)
{
   LTC_ARGCHK(pt != NULL);
   LTC_ARGCHK(ct != NULL);
   LTC_ARGCHK(f8 != NULL);
   return f8_encrypt(ct, pt, len, f8);
}


#endif

 

/* $Source: /cvs/libtom/libtomcrypt/src/modes/f8/f8_decrypt.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/06/16 22:49:25 $ */
