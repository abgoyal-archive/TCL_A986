
#include "tomcrypt.h"

#ifdef OCB_MODE

int ocb_done_encrypt(ocb_state *ocb, const unsigned char *pt, unsigned long ptlen,
                     unsigned char *ct, unsigned char *tag, unsigned long *taglen)
{
   LTC_ARGCHK(ocb    != NULL);
   LTC_ARGCHK(pt     != NULL);
   LTC_ARGCHK(ct     != NULL);
   LTC_ARGCHK(tag    != NULL);
   LTC_ARGCHK(taglen != NULL);
   return s_ocb_done(ocb, pt, ptlen, ct, tag, taglen, 0);
}

#endif


/* $Source: /cvs/libtom/libtomcrypt/src/encauth/ocb/ocb_done_encrypt.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
