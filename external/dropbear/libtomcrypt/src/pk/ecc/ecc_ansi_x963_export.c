
#include "tomcrypt.h"


#ifdef MECC

int ecc_ansi_x963_export(ecc_key *key, unsigned char *out, unsigned long *outlen)
{
   unsigned char buf[ECC_BUF_SIZE];
   unsigned long numlen;

   LTC_ARGCHK(key    != NULL);
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);

   if (ltc_ecc_is_valid_idx(key->idx) == 0) {
      return CRYPT_INVALID_ARG;
   }
   numlen = key->dp->size;

   if (*outlen < (1 + 2*numlen)) {
      *outlen = 1 + 2*numlen;
      return CRYPT_BUFFER_OVERFLOW;
   }

   /* store byte 0x04 */
   out[0] = 0x04;

   /* pad and store x */
   zeromem(buf, sizeof(buf));
   mp_to_unsigned_bin(key->pubkey.x, buf + (numlen - mp_unsigned_bin_size(key->pubkey.x)));
   XMEMCPY(out+1, buf, numlen);

   /* pad and store y */
   zeromem(buf, sizeof(buf));
   mp_to_unsigned_bin(key->pubkey.y, buf + (numlen - mp_unsigned_bin_size(key->pubkey.y)));
   XMEMCPY(out+1+numlen, buf, numlen);

   *outlen = 1 + 2*numlen;
   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_ansi_x963_export.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/12/04 02:50:11 $ */
