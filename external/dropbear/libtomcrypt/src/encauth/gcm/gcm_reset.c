
#include "tomcrypt.h"

#ifdef GCM_MODE

int gcm_reset(gcm_state *gcm)
{
   LTC_ARGCHK(gcm != NULL);

   zeromem(gcm->buf, sizeof(gcm->buf));
   zeromem(gcm->X,   sizeof(gcm->X));
   gcm->mode     = GCM_MODE_IV;
   gcm->ivmode   = 0;
   gcm->buflen   = 0;
   gcm->totlen   = 0;
   gcm->pttotlen = 0;
  
   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/gcm/gcm_reset.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
