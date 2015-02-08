#include "tomcrypt.h"


#ifdef LTC_DER

unsigned long der_utf8_charsize(const wchar_t c)
{
   if (c <= 0x7F) {
      return 1;
   } else if (c <= 0x7FF) {
      return 2;
   } else if (c <= 0xFFFF) {
      return 3;
   } else {
      return 4;
   }
}

int der_length_utf8_string(const wchar_t *in, unsigned long noctets, unsigned long *outlen)
{
   unsigned long x, len;

   LTC_ARGCHK(in     != NULL);
   LTC_ARGCHK(outlen != NULL);

   len = 0;
   for (x = 0; x < noctets; x++) {
      if (in[x] < 0 || in[x] > 0x10FFFF) {
         return CRYPT_INVALID_ARG;
      }
      len += der_utf8_charsize(in[x]);
   }

   if (len < 128) {
      /* 0C LL DD DD DD ... */
      *outlen = 2 + len;
   } else if (len < 256) {
      /* 0C 81 LL DD DD DD ... */
      *outlen = 3 + len;
   } else if (len < 65536UL) {
      /* 0C 82 LL LL DD DD DD ... */
      *outlen = 4 + len;
   } else if (len < 16777216UL) {
      /* 0C 83 LL LL LL DD DD DD ... */
      *outlen = 5 + len;
   } else {
      return CRYPT_INVALID_ARG;
   }

   return CRYPT_OK;
}

#endif


/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/utf8/der_length_utf8_string.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/12/16 17:41:21 $ */
