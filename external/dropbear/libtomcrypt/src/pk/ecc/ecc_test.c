
#include "tomcrypt.h"


#ifdef MECC

int ecc_test(void)
{
   void     *modulus, *order;
   ecc_point  *G, *GG;
   int i, err, primality;

   if ((err = mp_init_multi(&modulus, &order, NULL)) != CRYPT_OK) {
      return err;
   }

   G   = ltc_ecc_new_point();
   GG  = ltc_ecc_new_point();
   if (G == NULL || GG == NULL) {
      mp_clear_multi(modulus, order, NULL);
      ltc_ecc_del_point(G);
      ltc_ecc_del_point(GG);
      return CRYPT_MEM;
   }

   for (i = 0; ltc_ecc_sets[i].size; i++) {
       #if 0
          printf("Testing %d\n", ltc_ecc_sets[i].size);
       #endif
       if ((err = mp_read_radix(modulus, (char *)ltc_ecc_sets[i].prime, 16)) != CRYPT_OK)   { goto done; }
       if ((err = mp_read_radix(order, (char *)ltc_ecc_sets[i].order, 16)) != CRYPT_OK)     { goto done; }

       /* is prime actually prime? */
       if ((err = mp_prime_is_prime(modulus, 8, &primality)) != CRYPT_OK)                   { goto done; }
       if (primality == 0) {
          err = CRYPT_FAIL_TESTVECTOR;
          goto done;
       }

       /* is order prime ? */
       if ((err = mp_prime_is_prime(order, 8, &primality)) != CRYPT_OK)                     { goto done; }
       if (primality == 0) {
          err = CRYPT_FAIL_TESTVECTOR;
          goto done;
       }

       if ((err = mp_read_radix(G->x, (char *)ltc_ecc_sets[i].Gx, 16)) != CRYPT_OK)         { goto done; }
       if ((err = mp_read_radix(G->y, (char *)ltc_ecc_sets[i].Gy, 16)) != CRYPT_OK)         { goto done; }
       mp_set(G->z, 1);

       /* then we should have G == (order + 1)G */
       if ((err = mp_add_d(order, 1, order)) != CRYPT_OK)                                   { goto done; }
       if ((err = ltc_mp.ecc_ptmul(order, G, GG, modulus, 1)) != CRYPT_OK)                  { goto done; }
       if (mp_cmp(G->x, GG->x) != LTC_MP_EQ || mp_cmp(G->y, GG->y) != LTC_MP_EQ) {
          err = CRYPT_FAIL_TESTVECTOR;
          goto done;
       }
   }
   err = CRYPT_OK;
done:
   ltc_ecc_del_point(GG);
   ltc_ecc_del_point(G);
   mp_clear_multi(order, modulus, NULL);
   return err;
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ecc_test.c,v $ */
/* $Revision: 1.10 $ */
/* $Date: 2006/12/04 02:19:48 $ */

