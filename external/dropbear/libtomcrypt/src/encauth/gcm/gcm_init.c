
#include "tomcrypt.h"

#ifdef GCM_MODE

int gcm_init(gcm_state *gcm, int cipher, 
             const unsigned char *key,  int keylen)
{
   int           err;
   unsigned char B[16];
#ifdef GCM_TABLES
   int           x, y, z, t;
#endif

   LTC_ARGCHK(gcm != NULL);
   LTC_ARGCHK(key != NULL);

#ifdef LTC_FAST
   if (16 % sizeof(LTC_FAST_TYPE)) {
      return CRYPT_INVALID_ARG;
   }
#endif

   /* is cipher valid? */
   if ((err = cipher_is_valid(cipher)) != CRYPT_OK) {
      return err;
   }
   if (cipher_descriptor[cipher].block_length != 16) {
      return CRYPT_INVALID_CIPHER;
   }

   /* schedule key */
   if ((err = cipher_descriptor[cipher].setup(key, keylen, 0, &gcm->K)) != CRYPT_OK) {
      return err;
   }

   /* H = E(0) */
   zeromem(B, 16);
   if ((err = cipher_descriptor[cipher].ecb_encrypt(B, gcm->H, &gcm->K)) != CRYPT_OK) {
      return err;
   }

   /* setup state */
   zeromem(gcm->buf, sizeof(gcm->buf));
   zeromem(gcm->X,   sizeof(gcm->X));
   gcm->cipher   = cipher;
   gcm->mode     = GCM_MODE_IV;
   gcm->ivmode   = 0;
   gcm->buflen   = 0;
   gcm->totlen   = 0;
   gcm->pttotlen = 0;

#ifdef GCM_TABLES
   /* setup tables */

   /* generate the first table as it has no shifting (from which we make the other tables) */
   zeromem(B, 16);
   for (y = 0; y < 256; y++) {
        B[0] = y;
        gcm_gf_mult(gcm->H, B, &gcm->PC[0][y][0]);
   }

   /* now generate the rest of the tables based the previous table */
   for (x = 1; x < 16; x++) {
      for (y = 0; y < 256; y++) {
         /* now shift it right by 8 bits */
         t = gcm->PC[x-1][y][15];
         for (z = 15; z > 0; z--) {
             gcm->PC[x][y][z] = gcm->PC[x-1][y][z-1];
         }
         gcm->PC[x][y][0] = gcm_shift_table[t<<1];
         gcm->PC[x][y][1] ^= gcm_shift_table[(t<<1)+1];
     }
  }

#endif

   return CRYPT_OK;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/encauth/gcm/gcm_init.c,v $ */
/* $Revision: 1.18 $ */
/* $Date: 2006/03/31 14:15:35 $ */
