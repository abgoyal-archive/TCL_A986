#include "tomcrypt.h"


#ifdef MRSA

int rsa_decrypt_key_ex(const unsigned char *in,       unsigned long  inlen,
                             unsigned char *out,      unsigned long *outlen,
                       const unsigned char *lparam,   unsigned long  lparamlen,
                             int            hash_idx, int            padding,
                             int           *stat,     rsa_key       *key)
{
  unsigned long modulus_bitlen, modulus_bytelen, x;
  int           err;
  unsigned char *tmp;

  LTC_ARGCHK(out    != NULL);
  LTC_ARGCHK(outlen != NULL);
  LTC_ARGCHK(key    != NULL);
  LTC_ARGCHK(stat   != NULL);

  /* default to invalid */
  *stat = 0;

  /* valid padding? */

  if ((padding != LTC_PKCS_1_V1_5) &&
      (padding != LTC_PKCS_1_OAEP)) {
    return CRYPT_PK_INVALID_PADDING;
  }

  if (padding == LTC_PKCS_1_OAEP) {
    /* valid hash ? */
    if ((err = hash_is_valid(hash_idx)) != CRYPT_OK) {
       return err;
    }
  }

  /* get modulus len in bits */
  modulus_bitlen = mp_count_bits( (key->N));

  /* outlen must be at least the size of the modulus */
  modulus_bytelen = mp_unsigned_bin_size( (key->N));
  if (modulus_bytelen != inlen) {
     return CRYPT_INVALID_PACKET;
  }

  /* allocate ram */
  tmp = XMALLOC(inlen);
  if (tmp == NULL) {
     return CRYPT_MEM;
  }

  /* rsa decode the packet */
  x = inlen;
  if ((err = ltc_mp.rsa_me(in, inlen, tmp, &x, PK_PRIVATE, key)) != CRYPT_OK) {
     XFREE(tmp);
     return err;
  }

  if (padding == LTC_PKCS_1_OAEP) {
    /* now OAEP decode the packet */
    err = pkcs_1_oaep_decode(tmp, x, lparam, lparamlen, modulus_bitlen, hash_idx,
                             out, outlen, stat);
  } else {
    /* now PKCS #1 v1.5 depad the packet */
    err = pkcs_1_v1_5_decode(tmp, x, LTC_PKCS_1_EME, modulus_bitlen, out, outlen, stat);
  }

  XFREE(tmp);
  return err;
}

#endif /* MRSA */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/rsa/rsa_decrypt_key.c,v $ */
/* $Revision: 1.8 $ */
/* $Date: 2006/11/01 09:18:22 $ */
