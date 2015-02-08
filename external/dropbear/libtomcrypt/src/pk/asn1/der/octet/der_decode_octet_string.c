#include "tomcrypt.h"



#ifdef LTC_DER

int der_decode_octet_string(const unsigned char *in, unsigned long inlen,
                                  unsigned char *out, unsigned long *outlen)
{
   unsigned long x, y, len;

   LTC_ARGCHK(in     != NULL);
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);

   /* must have header at least */
   if (inlen < 2) {
      return CRYPT_INVALID_PACKET;
   }

   /* check for 0x04 */
   if ((in[0] & 0x1F) != 0x04) {
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

   /* is it too long? */
   if (len > *outlen) {
      *outlen = len;
      return CRYPT_BUFFER_OVERFLOW;
   }

   if (len + x > inlen) {
      return CRYPT_INVALID_PACKET;
   }

   /* read the data */
   for (y = 0; y < len; y++) {
       out[y] = in[x++];
   }

   *outlen = y;

   return CRYPT_OK;
}
 
#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/octet/der_decode_octet_string.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/06/16 21:53:41 $ */
