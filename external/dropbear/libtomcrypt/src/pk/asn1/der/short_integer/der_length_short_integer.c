#include "tomcrypt.h"



#ifdef LTC_DER
int der_length_short_integer(unsigned long num, unsigned long *outlen)
{
   unsigned long z, y, len;

   LTC_ARGCHK(outlen  != NULL);

   /* force to 32 bits */
   num &= 0xFFFFFFFFUL;

   /* get the number of bytes */
   z = 0;
   y = num;
   while (y) {
     ++z;
     y >>= 8;
   }
   
   /* handle zero */
   if (z == 0) {
      z = 1;
   }

   /* we need a 0x02 to indicate it's INTEGER */
   len = 1;

   /* length byte */
   ++len;

   /* bytes in value */
   len += z;

   /* see if msb is set */
   len += (num&(1UL<<((z<<3) - 1))) ? 1 : 0;

   /* return length */
   *outlen = len; 
   
   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/short_integer/der_length_short_integer.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
