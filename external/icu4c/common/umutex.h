
#ifndef UMUTEX_H
#define UMUTEX_H

#include "unicode/utypes.h"
#include "unicode/uclean.h"  


/* APP_NO_THREADS is an old symbol. We'll honour it if present. */
#ifdef APP_NO_THREADS
# define ICU_USE_THREADS 0
#endif

#ifndef ICU_USE_THREADS
# define ICU_USE_THREADS 1
#endif

#if !defined(UMTX_STRONG_MEMORY_MODEL)
#define UMTX_STRONG_MEMORY_MODEL 0
#endif

#if UMTX_STRONG_MEMORY_MODEL

#define UMTX_CHECK(pMutex, expression, result) \
    (result)=(expression)

#else

#define UMTX_CHECK(pMutex, expression, result) \
    umtx_lock(pMutex); \
    (result)=(expression); \
    umtx_unlock(pMutex)

#endif


U_CAPI void U_EXPORT2 umtx_lock   ( UMTX* mutex ); 

U_CAPI void U_EXPORT2 umtx_unlock ( UMTX* mutex );

U_CAPI void U_EXPORT2 umtx_init   ( UMTX* mutex );

U_CAPI void U_EXPORT2 umtx_destroy( UMTX *mutex );



U_CAPI int32_t U_EXPORT2 umtx_atomic_inc(int32_t *);
U_CAPI int32_t U_EXPORT2 umtx_atomic_dec(int32_t *);


#endif /*_CMUTEX*/
/*eof*/



