
#include "tomcrypt.h"

#if defined(GCM_MODE)
void gcm_mult_h(gcm_state *gcm, unsigned char *I)
{
   unsigned char T[16];
#ifdef GCM_TABLES
   int x, y;
#ifdef GCM_TABLES_SSE2
   asm("movdqa (%0),%%xmm0"::"r"(&gcm->PC[0][I[0]][0]));
   for (x = 1; x < 16; x++) {
      asm("pxor (%0),%%xmm0"::"r"(&gcm->PC[x][I[x]][0]));
   }
   asm("movdqa %%xmm0,(%0)"::"r"(&T));
#else
   XMEMCPY(T, &gcm->PC[0][I[0]][0], 16);
   for (x = 1; x < 16; x++) {
#ifdef LTC_FAST
       for (y = 0; y < 16; y += sizeof(LTC_FAST_TYPE)) {
           *((LTC_FAST_TYPE *)(T + y)) ^= *((LTC_FAST_TYPE *)(&gcm->PC[x][I[x]][y]));
       }
#else
       for (y = 0; y < 16; y++) {
           T[y] ^= gcm->PC[x][I[x]][y];
       }
#endif /* LTC_FAST */
   }
#endif /* GCM_TABLES_SSE2 */
#else     
   gcm_gf_mult(gcm->H, I, T); 
#endif
   XMEMCPY(I, T, 16);
}
#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/gcm/gcm_mult_h.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/08/23 20:40:23 $ */
