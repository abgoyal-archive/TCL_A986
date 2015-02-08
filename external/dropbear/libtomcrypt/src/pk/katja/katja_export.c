#include "tomcrypt.h"


#ifdef MKAT

int katja_export(unsigned char *out, unsigned long *outlen, int type, katja_key *key)
{
   int           err;
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
            Version, n, d, p, q, d mod (p-1), d mod (q - 1), 1/q mod p, pq
       */
      if ((err = der_encode_sequence_multi(out, outlen, 
                          LTC_ASN1_SHORT_INTEGER, 1UL, &zero, 
                          LTC_ASN1_INTEGER, 1UL,  key->N, 
                          LTC_ASN1_INTEGER, 1UL,  key->d, 
                          LTC_ASN1_INTEGER, 1UL,  key->p, 
                          LTC_ASN1_INTEGER, 1UL,  key->q, 
                          LTC_ASN1_INTEGER, 1UL,  key->dP,
                          LTC_ASN1_INTEGER, 1UL,  key->dQ, 
                          LTC_ASN1_INTEGER, 1UL,  key->qP, 
                          LTC_ASN1_INTEGER, 1UL,  key->pq, 
                          LTC_ASN1_EOL,     0UL, NULL)) != CRYPT_OK) {
         return err;
      }
 
      /* clear zero and return */
      return CRYPT_OK;
   } else {
      /* public key */
      return der_encode_sequence_multi(out, outlen, 
                                 LTC_ASN1_INTEGER, 1UL, key->N, 
                                 LTC_ASN1_EOL,     0UL, NULL);
   }
}

#endif /* MRSA */

/* $Source: /cvs/libtom/libtomcrypt/src/pk/katja/katja_export.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/03/31 14:15:35 $ */
