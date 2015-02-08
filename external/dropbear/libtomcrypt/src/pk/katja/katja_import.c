#include "tomcrypt.h"


#ifdef MKAT

int katja_import(const unsigned char *in, unsigned long inlen, katja_key *key)
{
   int           err;
   void         *zero;

   LTC_ARGCHK(in  != NULL);
   LTC_ARGCHK(key != NULL);
   LTC_ARGCHK(ltc_mp.name != NULL);

   /* init key */
   if ((err = mp_init_multi(&zero, &key->d, &key->N, &key->dQ, 
                            &key->dP, &key->qP, &key->p, &key->q, &key->pq, NULL)) != CRYPT_OK) {
      return err;
   }

   if ((err = der_decode_sequence_multi(in, inlen, 
                                  LTC_ASN1_INTEGER, 1UL, key->N, 
                                  LTC_ASN1_EOL,     0UL, NULL)) != CRYPT_OK) {
      goto LBL_ERR;
   }

   if (mp_cmp_d(key->N, 0) == LTC_MP_EQ) {
      /* it's a private key */
      if ((err = der_decode_sequence_multi(in, inlen, 
                          LTC_ASN1_INTEGER, 1UL, zero, 
                          LTC_ASN1_INTEGER, 1UL, key->N, 
                          LTC_ASN1_INTEGER, 1UL, key->d, 
                          LTC_ASN1_INTEGER, 1UL, key->p, 
                          LTC_ASN1_INTEGER, 1UL, key->q, 
                          LTC_ASN1_INTEGER, 1UL, key->dP,
                          LTC_ASN1_INTEGER, 1UL, key->dQ, 
                          LTC_ASN1_INTEGER, 1UL, key->qP, 
                          LTC_ASN1_INTEGER, 1UL, key->pq, 
                          LTC_ASN1_EOL,     0UL, NULL)) != CRYPT_OK) {
         goto LBL_ERR;
      }
      key->type = PK_PRIVATE;
   } else {
      /* public we have N */
      key->type = PK_PUBLIC;
   }
   mp_clear(zero);
   return CRYPT_OK;
LBL_ERR:
   mp_clear_multi(zero,    key->d, key->N, key->dQ, key->dP,
                  key->qP, key->p, key->q, key->pq, NULL);
   return err;
}

#endif /* MRSA */


/* $Source: /cvs/libtom/libtomcrypt/src/pk/katja/katja_import.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:15:35 $ */
