#include "tomcrypt.h"


#ifdef LTC_DER
int der_length_bit_string(unsigned long nbits, unsigned long *outlen)
{
   unsigned long nbytes;
   LTC_ARGCHK(outlen != NULL);

   /* get the number of the bytes */
   nbytes = (nbits >> 3) + ((nbits & 7) ? 1 : 0) + 1;
 
   if (nbytes < 128) {
      /* 03 LL PP DD DD DD ... */
      *outlen = 2 + nbytes;
   } else if (nbytes < 256) {
      /* 03 81 LL PP DD DD DD ... */
      *outlen = 3 + nbytes;
   } else if (nbytes < 65536) {
      /* 03 82 LL LL PP DD DD DD ... */
      *outlen = 4 + nbytes;
   } else {
      return CRYPT_INVALID_ARG;
   }

   return CRYPT_OK;
}

#endif


/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/bit/der_length_bit_string.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/03/31 14:15:35 $ */
