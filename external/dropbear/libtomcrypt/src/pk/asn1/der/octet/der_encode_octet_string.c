#include "tomcrypt.h"



#ifdef LTC_DER

int der_encode_octet_string(const unsigned char *in, unsigned long inlen,
                                  unsigned char *out, unsigned long *outlen)
{
   unsigned long x, y, len;
   int           err;

   LTC_ARGCHK(in     != NULL);
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);

   /* get the size */
   if ((err = der_length_octet_string(inlen, &len)) != CRYPT_OK) {
      return err; 
   }

   /* too big? */
   if (len > *outlen) {
      *outlen = len;
      return CRYPT_BUFFER_OVERFLOW;
   }

   /* encode the header+len */
   x = 0;
   out[x++] = 0x04;
   if (inlen < 128) {
      out[x++] = (unsigned char)inlen;
   } else if (inlen < 256) {
      out[x++] = 0x81;
      out[x++] = (unsigned char)inlen;
   } else if (inlen < 65536UL) {
      out[x++] = 0x82;
      out[x++] = (unsigned char)((inlen>>8)&255);
      out[x++] = (unsigned char)(inlen&255);
   } else if (inlen < 16777216UL) {
      out[x++] = 0x83;
      out[x++] = (unsigned char)((inlen>>16)&255);
      out[x++] = (unsigned char)((inlen>>8)&255);
      out[x++] = (unsigned char)(inlen&255);
   } else {
      return CRYPT_INVALID_ARG;
   }

   /* store octets */
   for (y = 0; y < inlen; y++) {
       out[x++] = in[y];
   }

   /* retun length */
   *outlen = x;

   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/octet/der_encode_octet_string.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/12/04 21:34:03 $ */
