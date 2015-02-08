#include "tomcrypt.h"

#ifdef PKCS_1

int pkcs_1_os2ip(void *n, unsigned char *in, unsigned long inlen)
{
   return mp_read_unsigned_bin(n, in, inlen);
}

#endif /* PKCS_1 */


/* $Source: /cvs/libtom/libtomcrypt/src/pk/pkcs1/pkcs_1_os2ip.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/03/31 14:15:35 $ */
