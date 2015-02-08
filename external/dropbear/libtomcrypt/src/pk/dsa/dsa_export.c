#include "tomcrypt.h"


#ifdef MDSA

int dsa_export(unsigned char *out, unsigned long *outlen, int type, dsa_key *key)
{
   unsigned char flags[1];

   LTC_ARGCHK(out    != NULL);
   LTC_ARGCHK(outlen != NULL);
   LTC_ARGCHK(key    != NULL);

   /* can we store the static header?  */
   if (type == PK_PRIVATE && key->type != PK_PRIVATE) {
      return CRYPT_PK_TYPE_MISMATCH;
   }

   if (type != PK_PUBLIC && type != PK_PRIVATE) {
      return CRYPT_INVALID_ARG;
   }

   flags[0] = (type != PK_PUBLIC) ? 1 : 0;

   if (type == PK_PRIVATE) {
      return der_encode_sequence_multi(out, outlen,
                                 LTC_ASN1_BIT_STRING,   1UL, flags,
                                 LTC_ASN1_INTEGER,      1UL, key->g,
                                 LTC_ASN1_INTEGER,      1UL, key->p,
                                 LTC_ASN1_INTEGER,      1UL, key->q,
                                 LTC_ASN1_INTEGER,      1UL, key->y,
                                 LTC_ASN1_INTEGER,      1UL, key->x,
                                 LTC_ASN1_EOL,          0UL, NULL);
   } else {
      return der_encode_sequence_multi(out, outlen,
                                 LTC_ASN1_BIT_STRING,   1UL, flags,
                                 LTC_ASN1_INTEGER,      1UL, key->g,
                                 LTC_ASN1_INTEGER,      1UL, key->p,
                                 LTC_ASN1_INTEGER,      1UL, key->q,
                                 LTC_ASN1_INTEGER,      1UL, key->y,
                                 LTC_ASN1_EOL,          0UL, NULL);
   }
}

#endif


/* $Source: /cvs/libtom/libtomcrypt/src/pk/dsa/dsa_export.c,v $ */
/* $Revision: 1.8 $ */
/* $Date: 2006/03/31 14:15:35 $ */
