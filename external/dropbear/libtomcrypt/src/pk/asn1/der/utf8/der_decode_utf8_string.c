#include "tomcrypt.h"



#ifdef LTC_DER

int der_decode_utf8_string(const unsigned char *in,  unsigned long inlen,
                                       wchar_t *out, unsigned long *outlen)
{
   wchar_t       tmp;
   unsigned long x, y, z, len;

   LTC_ARGCHK(in     != NULL);
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);

   /* must have header at least */
   if (inlen < 2) {
      return CRYPT_INVALID_PACKET;
   }

   /* check for 0x0C */
   if ((in[0] & 0x1F) != 0x0C) {
      return CRYPT_INVALID_PACKET;
   }
   x = 1;

   /* decode the length */
   if (in[x] & 0x80) {
      /* valid # of bytes in length are 1,2,3 */
      y = in[x] & 0x7F;
      if ((y == 0) || (y > 3) || ((x + y) > inlen)) {
         return CRYPT_INVALID_PACKET;
      }

      /* read the length in */
      len = 0;
      ++x;
      while (y--) {
         len = (len << 8) | in[x++];
      }
   } else {
      len = in[x++] & 0x7F;
   }

   if (len + x > inlen) {
      return CRYPT_INVALID_PACKET;
   }

   /* proceed to decode */
   for (y = 0; x < inlen; ) {
      /* get first byte */
      tmp = in[x++];
 
      /* count number of bytes */
      for (z = 0; (tmp & 0x80) && (z <= 4); z++, tmp = (tmp << 1) & 0xFF);
      
      if (z > 4 || (x + (z - 1) > inlen)) {
         return CRYPT_INVALID_PACKET;
      }

      /* decode, grab upper bits */
      tmp >>= z;

      /* grab remaining bytes */
      if (z > 1) { --z; }
      while (z-- != 0) {
         if ((in[x] & 0xC0) != 0x80) {
            return CRYPT_INVALID_PACKET;
         }
         tmp = (tmp << 6) | ((wchar_t)in[x++] & 0x3F);
      }

      if (y > *outlen) {
         *outlen = y;
         return CRYPT_BUFFER_OVERFLOW;
      }
      out[y++] = tmp;
   }
   *outlen = y;

   return CRYPT_OK;
}
 
#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/utf8/der_decode_utf8_string.c,v $ */
/* $Revision: 1.7 $ */
/* $Date: 2006/11/26 02:27:37 $ */
