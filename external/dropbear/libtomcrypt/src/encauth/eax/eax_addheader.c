#include "tomcrypt.h"

#ifdef EAX_MODE

int eax_addheader(eax_state *eax, const unsigned char *header, 
                  unsigned long length)
{
   LTC_ARGCHK(eax    != NULL);
   LTC_ARGCHK(header != NULL);
   return omac_process(&eax->headeromac, header, length);
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/eax/eax_addheader.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
