#include "tomcrypt.h"


#ifdef PKCS_1

int pkcs_1_v1_5_decode(const unsigned char *msg, 
                             unsigned long  msglen,
                                       int  block_type,
                             unsigned long  modulus_bitlen,
                             unsigned char *out, 
                             unsigned long *outlen,
                                       int *is_valid)
{
  unsigned long modulus_len, ps_len, i;
  int result;

  /* default to invalid packet */
  *is_valid = 0;

  modulus_len = (modulus_bitlen >> 3) + (modulus_bitlen & 7 ? 1 : 0);

  /* test message size */

  if ((msglen > modulus_len) || (modulus_len < 11)) {
    return CRYPT_PK_INVALID_SIZE;
  }

  /* separate encoded message */

  if ((msg[0] != 0x00) || (msg[1] != (unsigned char)block_type)) {
    result = CRYPT_INVALID_PACKET;
    goto bail;
  }

  if (block_type == LTC_PKCS_1_EME) {
    for (i = 2; i < modulus_len; i++) {
      /* separator */
      if (msg[i] == 0x00) { break; }
    }
    ps_len = i++ - 2;

    if ((i >= modulus_len) || (ps_len < 8)) {
      /* There was no octet with hexadecimal value 0x00 to separate ps from m,
       * or the length of ps is less than 8 octets.
       */
      result = CRYPT_INVALID_PACKET;
      goto bail;
    }
  } else {
    for (i = 2; i < modulus_len - 1; i++) {
       if (msg[i] != 0xFF) { break; }
    }

    /* separator check */
    if (msg[i] != 0) {
      /* There was no octet with hexadecimal value 0x00 to separate ps from m. */
      result = CRYPT_INVALID_PACKET;
      goto bail;
    }

    ps_len = i - 2;
  }

  if (*outlen < (msglen - (2 + ps_len + 1))) {
    *outlen = msglen - (2 + ps_len + 1);
    result = CRYPT_BUFFER_OVERFLOW;
    goto bail;
  }

  *outlen = (msglen - (2 + ps_len + 1));
  XMEMCPY(out, &msg[2 + ps_len + 1], *outlen);

  /* valid packet */
  *is_valid = 1;
  result    = CRYPT_OK;
bail:
  return result;
} /* pkcs_1_v1_5_decode */

#endif /* #ifdef PKCS_1 */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/pkcs1/pkcs_1_v1_5_decode.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/12/16 17:41:21 $ */
