#include "tomcrypt.h"


#ifdef LTC_OFB_MODE

int ofb_decrypt(const unsigned char *ct, unsigned char *pt, unsigned long len, symmetric_OFB *ofb)
{
   LTC_ARGCHK(pt != NULL);
   LTC_ARGCHK(ct != NULL);
   LTC_ARGCHK(ofb != NULL);
   return ofb_encrypt(ct, pt, len, ofb);
}


#endif

 

/* $Source: /cvs/libtom/libtomcrypt/src/modes/ofb/ofb_decrypt.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/06/29 01:51:34 $ */
