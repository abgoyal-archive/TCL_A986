#include "tomcrypt.h"



#ifdef LTC_DER

int der_decode_integer(const unsigned char *in, unsigned long inlen, void *num)
{
   unsigned long x, y, z;
   int           err;

   LTC_ARGCHK(num    != NULL);
   LTC_ARGCHK(in     != NULL);

   /* min DER INTEGER is 0x02 01 00 == 0 */
   if (inlen < (1 + 1 + 1)) {
      return CRYPT_INVALID_PACKET;
   }

   /* ok expect 0x02 when we AND with 0001 1111 [1F] */
   x = 0;
   if ((in[x++] & 0x1F) != 0x02) {
      return CRYPT_INVALID_PACKET;
   }

   /* now decode the len stuff */
   z = in[x++];

   if ((z & 0x80) == 0x00) {
      /* short form */

      /* will it overflow? */
      if (x + z > inlen) {
         return CRYPT_INVALID_PACKET;
      }
     
      /* no so read it */
      if ((err = mp_read_unsigned_bin(num, (unsigned char *)in + x, z)) != CRYPT_OK) {
         return err;
      }
   } else {
      /* long form */
      z &= 0x7F;
      
      /* will number of length bytes overflow? (or > 4) */
      if (((x + z) > inlen) || (z > 4) || (z == 0)) {
         return CRYPT_INVALID_PACKET;
      }

      /* now read it in */
      y = 0;
      while (z--) {
         y = ((unsigned long)(in[x++])) | (y << 8);
      }

      /* now will reading y bytes overrun? */
      if ((x + y) > inlen) {
         return CRYPT_INVALID_PACKET;
      }

      /* no so read it */
      if ((err = mp_read_unsigned_bin(num, (unsigned char *)in + x, y)) != CRYPT_OK) {
         return err;
      }
   }

   /* see if it's negative */
   if (in[x] & 0x80) {
      void *tmp;
      if (mp_init(&tmp) != CRYPT_OK) {
         return CRYPT_MEM;
      }

      if (mp_2expt(tmp, mp_count_bits(num)) != CRYPT_OK || mp_sub(num, tmp, num) != CRYPT_OK) {
         mp_clear(tmp);
         return CRYPT_MEM;
      }
      mp_clear(tmp);
   } 

   return CRYPT_OK;

}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/integer/der_decode_integer.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
