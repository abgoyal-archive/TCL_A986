#include "tomcrypt.h"


void zeromem(void *out, size_t outlen)
{
   unsigned char *mem = out;
   LTC_ARGCHKVD(out != NULL);
   while (outlen-- > 0) {
      *mem++ = 0;
   }
}

/* $Source: /cvs/libtom/libtomcrypt/src/misc/zeromem.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/06/09 01:38:13 $ */
