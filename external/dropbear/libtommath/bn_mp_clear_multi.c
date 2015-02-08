#include <tommath.h>
#ifdef BN_MP_CLEAR_MULTI_C
#include <stdarg.h>

void mp_clear_multi(mp_int *mp, ...) 
{
    mp_int* next_mp = mp;
    va_list args;
    va_start(args, mp);
    while (next_mp != NULL) {
        mp_clear(next_mp);
        next_mp = va_arg(args, mp_int*);
    }
    va_end(args);
}
#endif

/* $Source: /cvs/libtom/libtommath/bn_mp_clear_multi.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:18:44 $ */
