
#include "tomcrypt.h"


#ifdef MECC

int ecc_ansi_x963_import(const unsigned char *in, unsigned long inlen, ecc_key *key)
{
   return ecc_ansi_x963_import_ex(in, inlen, key, NULL);
}

int ecc_ansi_x963_import_ex(const unsigned char *in, unsigned long inlen, ecc_key *key, ltc_ecc_set_type *dp)
{
   int x, err;
 
   LTC_ARGCHK(in  != NULL);
   LTC_ARGCHK(key != NULL);
   
   /* must be odd */
   if ((inlen & 1) == 0) {
      return CRYPT_INVALID_ARG;
   }

   /* init key */
   if (mp_init_multi(&key->pubkey.x, &key->pubkey.y, &key->pubkey.z, &key->k, NULL) != CRYPT_OK) {
      return CRYPT_MEM;
   }

   /* check for 4, 6 or 7 */
   if (in[0] != 4 && in[0] != 6 && in[0] != 7) {
      err = CRYPT_INVALID_PACKET;
      goto error;
   }

   /* read data */
   if ((err = mp_read_unsigned_bin(key->pubkey.x, (unsigned char *)in+1, (inlen-1)>>1)) != CRYPT_OK) {
      goto error;
   }

   if ((err = mp_read_unsigned_bin(key->pubkey.y, (unsigned char *)in+1+((inlen-1)>>1), (inlen-1)>>1)) != CRYPT_OK) {
      goto error;
   }
   if ((err = mp_set(key->pubkey.z, 1)) != CRYPT_OK) { goto error; }

   if (dp == NULL) {
     /* determine the idx */
      for (x = 0; ltc_ecc_sets[x].size != 0; x++) {
         if ((unsigned)ltc_ecc_sets[x].size >= ((inlen-1)>>1)) {
            break;
         }
      }
      if (ltc_ecc_sets[x].size == 0) {
         err = CRYPT_INVALID_PACKET;
         goto error;
      }
      /* set the idx */
      key->idx  = x;
      key->dp = &ltc_ecc_sets[x];
   } else {
      if (((inlen-1)>>1) != (unsigned long) dp->size) {
         err = CRYPT_INVALID_PACKET;
         goto error;
      }
      key->idx = -1;
      key->dp  = dp;
   }
   key->type = PK_PUBLIC;

   /* we're done */
   return CRYPT_OK;
error:
   mp_clear_multi(key->pubkey.x, key->pubkey.y, key->pubkey.z, key->k, NULL);
   return err;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_ansi_x963_import.c,v $ */
/* $Revision: 1.9 $ */
/* $Date: 2006/12/04 22:17:46 $ */
