#include "tomcrypt.h"


#ifdef LTC_DER

int der_length_utctime(ltc_utctime *utctime, unsigned long *outlen)
{
   LTC_ARGCHK(outlen  != NULL);
   LTC_ARGCHK(utctime != NULL);

   if (utctime->off_hh == 0 && utctime->off_mm == 0) {
      /* we encode as YYMMDDhhmmssZ */
      *outlen = 2 + 13;
   } else {
      /* we encode as YYMMDDhhmmss{+|-}hh'mm' */
      *outlen = 2 + 17;
   }

   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/utctime/der_length_utctime.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
