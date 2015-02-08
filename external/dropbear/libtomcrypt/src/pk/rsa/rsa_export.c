#include "tomcrypt.h"


#ifdef MRSA

int rsa_export(unsigned char *out, unsigned long *outlen, int type, rsa_key *key)
{
   unsigned long zero=0;
   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);
   LTC_ARGCHK(key    != NULL);

   /* type valid? */
   if (!(key->type == PK_PRIVATE) && (type == PK_PRIVATE)) {
      return CRYPT_PK_INVALID_TYPE;
   }

   if (type == PK_PRIVATE) {
      /* private key */
      /* output is 
            Version, n, e, d, p, q, d mod (p-1), d mod (q - 1), 1/q mod p
       */
      return der_encode_sequence_multi(out, outlen, 
                          LTC_ASN1_SHORT_INTEGER, 1UL, &zero, 
                          LTC_ASN1_INTEGER, 1UL,  key->N, 
                          LTC_ASN1_INTEGER, 1UL,  key->e,
                          LTC_ASN1_INTEGER, 1UL,  key->d, 
                          LTC_ASN1_INTEGER, 1UL,  key->p, 
                          LTC_ASN1_INTEGER, 1UL,  key->q, 
                          LTC_ASN1_INTEGER, 1UL,  key->dP,
                          LTC_ASN1_INTEGER, 1UL,  key->dQ, 
                          LTC_ASN1_INTEGER, 1UL,  key->qP, 
                          LTC_ASN1_EOL,     0UL, NULL);
   } else {
      /* public key */
      return der_encode_sequence_multi(out, outlen, 
                                 LTC_ASN1_INTEGER, 1UL,  key->N, 
                                 LTC_ASN1_INTEGER, 1UL,  key->e, 
                                 LTC_ASN1_EOL,     0UL, NULL);
   }
}

#endif /* MRSA */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/rsa/rsa_export.c,v $ */
/* $Revision: 1.15 $ */
/* $Date: 2006/03/31 14:15:35 $ */
