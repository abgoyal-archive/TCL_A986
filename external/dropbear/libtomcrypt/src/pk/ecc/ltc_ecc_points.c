
#include "tomcrypt.h"


#ifdef MECC

ecc_point *ltc_ecc_new_point(void)
{
   ecc_point *p;
   p = XCALLOC(1, sizeof(*p));
   if (p == NULL) {
      return NULL;
   }
   if (mp_init_multi(&p->x, &p->y, &p->z, NULL) != CRYPT_OK) {
      XFREE(p);
      return NULL;
   }
   return p;
}

void ltc_ecc_del_point(ecc_point *p)
{
   /* prevents free'ing null arguments */
   if (p != NULL) {
      mp_clear_multi(p->x, p->y, p->z, NULL); /* note: p->z may be NULL but that's ok with this function anyways */
      XFREE(p);
   }
}

#endif
/* $Source: /cvs/libtom/libtomcrypt/src/pk/ecc/ltc_ecc_points.c,v $ */
/* $Revision: 1.5 $ */
/* $Date: 2006/12/04 02:19:48 $ */

