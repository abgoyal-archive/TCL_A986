#include "tomcrypt.h"


#ifdef LTC_DER
int der_length_boolean(unsigned long *outlen)
{
   LTC_ARGCHK(outlen != NULL);
   *outlen = 3;
   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/boolean/der_length_boolean.c,v $ */
/* $Revision: 1.2 $ */
/* $Date: 2006/04/22 17:28:38 $ */
