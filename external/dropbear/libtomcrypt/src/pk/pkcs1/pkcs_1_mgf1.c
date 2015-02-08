#include "tomcrypt.h"


#ifdef PKCS_1

int pkcs_1_mgf1(int                  hash_idx,
                const unsigned char *seed, unsigned long seedlen,
                      unsigned char *mask, unsigned long masklen)
{
   unsigned long hLen, x;
   ulong32       counter;
   int           err;
   hash_state    *md;
   unsigned char *buf;
 
   LTC_ARGCHK(seed != NULL);
   LTC_ARGCHK(mask != NULL);

   /* ensure valid hash */
   if ((err = hash_is_valid(hash_idx)) != CRYPT_OK) { 
      return err;
   }

   /* get hash output size */
   hLen = hash_descriptor[hash_idx].hashsize;

   /* allocate memory */
   md  = XMALLOC(sizeof(hash_state));
   buf = XMALLOC(hLen);
   if (md == NULL || buf == NULL) {
      if (md != NULL) {
         XFREE(md);
      }
      if (buf != NULL) {
         XFREE(buf);
      }
      return CRYPT_MEM;
   }

   /* start counter */
   counter = 0;

   while (masklen > 0) {
       /* handle counter */
       STORE32H(counter, buf);
       ++counter;

       /* get hash of seed || counter */
       if ((err = hash_descriptor[hash_idx].init(md)) != CRYPT_OK) {
          goto LBL_ERR;
       }
       if ((err = hash_descriptor[hash_idx].process(md, seed, seedlen)) != CRYPT_OK) {
          goto LBL_ERR;
       }
       if ((err = hash_descriptor[hash_idx].process(md, buf, 4)) != CRYPT_OK) {
          goto LBL_ERR;
       }
       if ((err = hash_descriptor[hash_idx].done(md, buf)) != CRYPT_OK) {
          goto LBL_ERR;
       }

       /* store it */
       for (x = 0; x < hLen && masklen > 0; x++, masklen--) {
          *mask++ = buf[x];
       }
   }

   err = CRYPT_OK;
LBL_ERR:
#ifdef LTC_CLEAN_STACK
   zeromem(buf, hLen);
   zeromem(md,  sizeof(hash_state));
#endif

   XFREE(buf);
   XFREE(md);

   return err;
}

#endif /* PKCS_1 */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/pkcs1/pkcs_1_mgf1.c,v $ */
/* $Revision: 1.6 $ */
/* $Date: 2006/03/31 14:15:35 $ */
