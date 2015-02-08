#include "tomcrypt.h"


#ifdef MKAT

int katja_encrypt_key(const unsigned char *in,     unsigned long inlen,
                          unsigned char *out,    unsigned long *outlen,
                    const unsigned char *lparam, unsigned long lparamlen,
                    prng_state *prng, int prng_idx, int hash_idx, katja_key *key)
{
  unsigned long modulus_bitlen, modulus_bytelen, x;
  int           err;
  
  LTC_ARGCHK(in     != NULL);
  LTC_ARGCHK(out    != NULL);
  LTC_ARGCHK(outlen != NULL);
  LTC_ARGCHK(key    != NULL);
  
  /* valid prng and hash ? */
  if ((err = prng_is_valid(prng_idx)) != CRYPT_OK) {
     return err;
  }
  if ((err = hash_is_valid(hash_idx)) != CRYPT_OK) {
     return err;
  }
  
  /* get modulus len in bits */
  modulus_bitlen = mp_count_bits((key->N));

  /* payload is upto pq, so we know q is 1/3rd the size of N and therefore pq is 2/3th the size */
  modulus_bitlen = ((modulus_bitlen << 1) / 3);

  /* round down to next byte */
  modulus_bitlen -= (modulus_bitlen & 7) + 8;

  /* outlen must be at least the size of the modulus */
  modulus_bytelen = mp_unsigned_bin_size((key->N));
  if (modulus_bytelen > *outlen) {
     *outlen = modulus_bytelen;
     return CRYPT_BUFFER_OVERFLOW;
  }

  /* OAEP pad the key */
  x = *outlen;
  if ((err = pkcs_1_oaep_encode(in, inlen, lparam, 
                                lparamlen, modulus_bitlen, prng, prng_idx, hash_idx, 
                                out, &x)) != CRYPT_OK) {
     return err;
  }                          

  /* Katja exptmod the OAEP pad */
  return katja_exptmod(out, x, out, outlen, PK_PUBLIC, key);
}

#endif /* MRSA */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/katja/katja_encrypt_key.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/06/16 21:53:41 $ */
