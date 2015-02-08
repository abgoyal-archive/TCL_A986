#include "tomcrypt.h"


#ifdef PELICAN

int pelican_memory(const unsigned char *key, unsigned long keylen,
                   const unsigned char *in,  unsigned long inlen,
                         unsigned char *out)
{
   pelican_state *pel;
   int err;

   pel = XMALLOC(sizeof(*pel));
   if (pel == NULL) { 
      return CRYPT_MEM;
   }

   if ((err = pelican_init(pel, key, keylen)) != CRYPT_OK) {
      XFREE(pel);
      return err;
   }
   if ((err = pelican_process(pel, in ,inlen)) != CRYPT_OK) {
      XFREE(pel);
      return err;
   }
   err = pelican_done(pel, out);
   XFREE(pel); 
   return err;
}


#endif

/* $Source: /cvs/libtom/libtomcrypt/src/mac/pelican/pelican_memory.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/03/31 14:15:35 $ */
