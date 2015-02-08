#include "tomcrypt.h"


#ifdef MRSA

int rsa_encrypt_key_ex(const unsigned char *in,     unsigned long inlen,
                             unsigned char *out,    unsigned long *outlen,
                       const unsigned char *lparam, unsigned long lparamlen,
                       prng_state *prng, int prng_idx, int hash_idx, int padding, rsa_key *key)
{
  unsigned long modulus_bitlen, modulus_bytelen, x;
  int           err;

  LTC_ARGCHK(in     != NULL);
  LTC_ARGCHK(out    != NULL);
  LTC_ARGCHK(outlen != NULL);
  LTC_ARGCHK(key    != NULL);

  /* valid padding? */
  if ((padding != LTC_PKCS_1_V1_5) &&
      (padding != LTC_PKCS_1_OAEP)) {
    return CRYPT_PK_INVALID_PADDING;
  }

  /* valid prng? */
  if ((err = prng_is_valid(prng_idx)) != CRYPT_OK) {
     return err;
  }

  if (padding == LTC_PKCS_1_OAEP) {
    /* valid hash? */
    if ((err = hash_is_valid(hash_idx)) != CRYPT_OK) {
       return err;
    }
  }

  /* get modulus len in bits */
  modulus_bitlen = mp_count_bits( (key->N));

  /* outlen must be at least the size of the modulus */
  modulus_bytelen = mp_unsigned_bin_size( (key->N));
  if (modulus_bytelen > *outlen) {
     *outlen = modulus_bytelen;
     return CRYPT_BUFFER_OVERFLOW;
  }

  if (padding == LTC_PKCS_1_OAEP) {
    /* OAEP pad the key */
    x = *outlen;
    if ((err = pkcs_1_oaep_encode(in, inlen, lparam,
                                  lparamlen, modulus_bitlen, prng, prng_idx, hash_idx,
                                  out, &x)) != CRYPT_OK) {
       return err;
    }
  } else {
    /* PKCS #1 v1.5 pad the key */
    x = *outlen;
    if ((err = pkcs_1_v1_5_encode(in, inlen, LTC_PKCS_1_EME,
                                  modulus_bitlen, prng, prng_idx,
                                  out, &x)) != CRYPT_OK) {
      return err;
    }
  }

  /* rsa exptmod the OAEP or PKCS #1 v1.5 pad */
  return ltc_mp.rsa_me(out, x, out, outlen, PK_PUBLIC, key);
}

#endif /* MRSA */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/rsa/rsa_encrypt_key.c,v $ */
/* $Revision: 1.8 $ */
/* $Date: 2006/11/01 09:18:22 $ */
