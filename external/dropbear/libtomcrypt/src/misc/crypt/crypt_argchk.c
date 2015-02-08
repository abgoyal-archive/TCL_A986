#include "tomcrypt.h"
#include <signal.h>


#if (ARGTYPE == 0)
void crypt_argchk(char *v, char *s, int d)
{
 fprintf(stderr, "LTC_ARGCHK '%s' failure on line %d of file %s\n",
         v, d, s);
 (void)raise(SIGABRT);
}
#endif

/* $Source: /cvs/libtom/libtomcrypt/src/misc/crypt/crypt_argchk.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
