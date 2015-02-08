#include "tomcrypt.h"
#include <stdarg.h>


#ifdef LTC_OMAC

int omac_memory_multi(int cipher, 
                const unsigned char *key, unsigned long keylen,
                      unsigned char *out, unsigned long *outlen,
                const unsigned char *in,  unsigned long inlen, ...)
{
   int                  err;
   omac_state          *omac;
   va_list              args;
   const unsigned char *curptr;
   unsigned long        curlen;

   LTC_ARGCHK(key    != NULL);
   LTC_ARGCHK(in     != NULL);
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);

   /* allocate ram for omac state */
   omac = XMALLOC(sizeof(omac_state));
   if (omac == NULL) {
      return CRYPT_MEM;
   }

   /* omac process the message */
   if ((err = omac_init(omac, cipher, key, keylen)) != CRYPT_OK) {
      goto LBL_ERR;
   }
   va_start(args, inlen);
   curptr = in; 
   curlen = inlen;
   for (;;) {
      /* process buf */
      if ((err = omac_process(omac, curptr, curlen)) != CRYPT_OK) {
         goto LBL_ERR;
      }
      /* step to next */
      curptr = va_arg(args, const unsigned char*);
      if (curptr == NULL) {
         break;
      }
      curlen = va_arg(args, unsigned long);
   }
   if ((err = omac_done(omac, out, outlen)) != CRYPT_OK) {
      goto LBL_ERR;
   }
LBL_ERR:
#ifdef LTC_CLEAN_STACK
   zeromem(omac, sizeof(omac_state));
#endif
   XFREE(omac);
   va_end(args);
   return err;   
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/mac/omac/omac_memory_multi.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/11/03 00:39:49 $ */
