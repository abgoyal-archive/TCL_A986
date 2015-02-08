#include <tommath.h>
#ifdef BNCORE_C


int     KARATSUBA_MUL_CUTOFF = 80,      /* Min. number of digits before Karatsuba multiplication is used. */
        KARATSUBA_SQR_CUTOFF = 120,     /* Min. number of digits before Karatsuba squaring is used. */
        
        TOOM_MUL_CUTOFF      = 350,      /* no optimal values of these are known yet so set em high */
        TOOM_SQR_CUTOFF      = 400; 
#endif

/* $Source: /cvs/libtom/libtommath/bncore.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:18:44 $ */
