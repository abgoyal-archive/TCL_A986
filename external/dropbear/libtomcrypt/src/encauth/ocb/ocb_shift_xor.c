
#include "tomcrypt.h"

#ifdef OCB_MODE

void ocb_shift_xor(ocb_state *ocb, unsigned char *Z)
{
   int x, y;
   y = ocb_ntz(ocb->block_index++);
   for (x = 0; x < ocb->block_len; x++) {
       ocb->Li[x] ^= ocb->Ls[y][x];
       Z[x]        = ocb->Li[x] ^ ocb->R[x];
   }
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/ocb/ocb_shift_xor.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
