
#include "unicode/utypes.h"
#include "uassert.h"
#include "ucln_cmn.h"

#if defined(U_DARWIN)
#include <AvailabilityMacros.h>
#if (ICU_USE_THREADS == 1) && defined(MAC_OS_X_VERSION_10_4) && defined(MAC_OS_X_VERSION_MIN_REQUIRED) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
#if defined(__STRICT_ANSI__)
#define UPRV_REMAP_INLINE
#define inline
#endif
#include <libkern/OSAtomic.h>
#define USE_MAC_OS_ATOMIC_INCREMENT 1
#if defined(UPRV_REMAP_INLINE)
#undef inline
#undef UPRV_REMAP_INLINE
#endif
#endif
#endif

/* Assume POSIX, and modify as necessary below */
#define POSIX

#if defined(U_WINDOWS)
#undef POSIX
#endif
#if defined(macintosh)
#undef POSIX
#endif
#if defined(OS2)
#undef POSIX
#endif

#if defined(POSIX) && (ICU_USE_THREADS==1)
# include <pthread.h> /* must be first, so that we get the multithread versions of things. */

#endif /* POSIX && (ICU_USE_THREADS==1) */

#ifdef U_WINDOWS
# define WIN32_LEAN_AND_MEAN
# define VC_EXTRALEAN
# define NOUSER
# define NOSERVICE
# define NOIME
# define NOMCX
# include <windows.h>
#endif

#include "umutex.h"
#include "cmemory.h"


#define  MAX_MUTEXES  40
static UMTX              gGlobalMutex          = NULL;
static UMTX              gIncDecMutex          = NULL;       
#if (ICU_USE_THREADS == 1)
static UBool             gMutexPoolInitialized = FALSE;
static char              gMutexesInUse[MAX_MUTEXES];   

#if defined(U_WINDOWS) 
static CRITICAL_SECTION  gMutexes[MAX_MUTEXES];
static CRITICAL_SECTION  gGlobalWinMutex;


#if defined(U_DEBUG) && (ICU_USE_THREADS==1)
static int32_t gRecursionCount = 0; /* detect global mutex locking */      
static int32_t gRecursionCountPool[MAX_MUTEXES]; /* ditto for non-global */
#endif


#elif defined(POSIX) 
static pthread_mutex_t   gMutexes[MAX_MUTEXES] = {
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER
};

#else 
static void *gMutexes[MAX_MUTEXES] = {
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, NULL };

#if (ICU_USE_THREADS == 1)
#error no ICU mutex implementation for this platform
#endif
#endif
#endif /* ICU_USE_THREADS==1 */




static UMtxInitFn    *pMutexInitFn    = NULL;
static UMtxFn        *pMutexDestroyFn = NULL;
static UMtxFn        *pMutexLockFn    = NULL;
static UMtxFn        *pMutexUnlockFn  = NULL;
static const void    *gMutexContext   = NULL;



U_CAPI void  U_EXPORT2
umtx_lock(UMTX *mutex)
{
    if (mutex == NULL) {
        mutex = &gGlobalMutex;
    }

    if (*mutex == NULL) {
        /* Lock of an uninitialized mutex.  Initialize it before proceeding.   */
        umtx_init(mutex);    
    }

    if (pMutexLockFn != NULL) {
        (*pMutexLockFn)(gMutexContext, mutex);
    } else {

#if (ICU_USE_THREADS == 1)
#if defined(U_WINDOWS)
        EnterCriticalSection((CRITICAL_SECTION*) *mutex);
#elif defined(POSIX)
        pthread_mutex_lock((pthread_mutex_t*) *mutex);
#endif   /* cascade of platforms */
#endif /* ICU_USE_THREADS==1 */
    }

#if defined(U_WINDOWS) && defined(U_DEBUG) && (ICU_USE_THREADS==1)
    if (mutex == &gGlobalMutex) {         /* Detect Reentrant locking of the global mutex.      */
        gRecursionCount++;                /* Recursion causes deadlocks on Unixes.              */
        U_ASSERT(gRecursionCount == 1);   /* Detection works on Windows.  Debug problems there. */
    }
    /* This handles gGlobalMutex too, but only if there is no pMutexLockFn */
    else if (pMutexLockFn == NULL) { /* see comments above */
        size_t i = ((CRITICAL_SECTION*)*mutex) - &gMutexes[0];
        U_ASSERT(i >= 0 && i < MAX_MUTEXES);
        ++gRecursionCountPool[i];

        U_ASSERT(gRecursionCountPool[i] == 1); /* !Detect Deadlock! */

        /* This works and is fast, but needs testing on Win98/NT/2K.
           See comments above. [alan]
          U_ASSERT((CRITICAL_SECTION*)*mutex >= &gMutexes[0] &&
                   (CRITICAL_SECTION*)*mutex <= &gMutexes[MAX_MUTEXES]);
          U_ASSERT(((CRITICAL_SECTION*)*mutex)->RecursionCount == 1);
        */
    }
#endif /*U_DEBUG*/
}



U_CAPI void  U_EXPORT2
umtx_unlock(UMTX* mutex)
{
    if(mutex == NULL) {
        mutex = &gGlobalMutex;
    }

    if(*mutex == NULL)    {
#if (ICU_USE_THREADS == 1)
        U_ASSERT(FALSE);  /* This mutex is not initialized.     */
#endif
        return; 
    }

#if defined (U_WINDOWS) && defined (U_DEBUG) && (ICU_USE_THREADS==1)
    if (mutex == &gGlobalMutex) {
        gRecursionCount--;
        U_ASSERT(gRecursionCount == 0);  /* Detect unlock of an already unlocked mutex */
    }
    /* This handles gGlobalMutex too, but only if there is no pMutexLockFn */
    else if (pMutexLockFn == NULL) { /* see comments above */
        size_t i = ((CRITICAL_SECTION*)*mutex) - &gMutexes[0];
        U_ASSERT(i >= 0 && i < MAX_MUTEXES);
        --gRecursionCountPool[i];

        U_ASSERT(gRecursionCountPool[i] == 0); /* !Detect Deadlock! */

        /* This works and is fast, but needs testing on Win98/NT/2K.
           Note that RecursionCount will be 1, not 0, since we haven't
           left the CRITICAL_SECTION yet.  See comments above. [alan]
          U_ASSERT((CRITICAL_SECTION*)*mutex >= &gMutexes[0] &&
                   (CRITICAL_SECTION*)*mutex <= &gMutexes[MAX_MUTEXES]);
          U_ASSERT(((CRITICAL_SECTION*)*mutex)->RecursionCount == 1);
        */
    }
#endif

    if (pMutexUnlockFn) {
        (*pMutexUnlockFn)(gMutexContext, mutex);
    } else {
#if (ICU_USE_THREADS==1)
#if defined (U_WINDOWS)
        LeaveCriticalSection((CRITICAL_SECTION*)*mutex);
#elif defined (POSIX)
        pthread_mutex_unlock((pthread_mutex_t*)*mutex);
#endif  /* cascade of platforms */
#endif /* ICU_USE_THREADS == 1 */
    }
}




static void initGlobalMutex() {
    /*
     * If User Supplied mutex functions are in use
     *    init the icu global mutex using them.  
     */
    if (pMutexInitFn != NULL) {
        if (gGlobalMutex==NULL) {
            UErrorCode status = U_ZERO_ERROR;
            (*pMutexInitFn)(gMutexContext, &gGlobalMutex, &status);
            if (U_FAILURE(status)) {
                /* TODO:  how should errors here be handled? */
                return;
            }
        }
        return;
    }

    /* No user override of mutex functions.
     *   Use default ICU mutex implementations.
     */
#if (ICU_USE_THREADS == 1)
    /*
     *  for Windows, init the pool of critical sections that we
     *    will use as needed for ICU mutexes.
     */
#if defined (U_WINDOWS)
    if (gMutexPoolInitialized == FALSE) {
        int i;
        for (i=0; i<MAX_MUTEXES; i++) {
            InitializeCriticalSection(&gMutexes[i]);
#if defined (U_DEBUG)
            gRecursionCountPool[i] = 0; /* see comments above */
#endif
        }
        gMutexPoolInitialized = TRUE;
    }
#elif defined (U_DARWIN)
    /* PTHREAD_MUTEX_INITIALIZER works, don't need to call pthread_mutex_init
     * as below (which is subject to a race condition)
     */
    gMutexPoolInitialized = TRUE;
#elif defined (POSIX)
    /*  TODO:  experimental code.  Shouldn't need to explicitly init the mutexes. */
    if (gMutexPoolInitialized == FALSE) {
        int i;
        for (i=0; i<MAX_MUTEXES; i++) {
            pthread_mutex_init(&gMutexes[i], NULL);
        }
        gMutexPoolInitialized = TRUE;
    }
#endif 

    /*
     * for both Windows & POSIX, the first mutex in the array is used
     *   for the ICU global mutex.
     */
    gGlobalMutex = &gMutexes[0];
    gMutexesInUse[0] = 1;

#else  /* ICU_USE_THREADS */
    gGlobalMutex = &gGlobalMutex;  /* With no threads, we must still set the mutex to
                                    * some non-null value to make the rest of the
                                    *   (not ifdefed) mutex code think that it is initialized.
                                    */
#endif /* ICU_USE_THREADS */
}





U_CAPI void  U_EXPORT2
umtx_init(UMTX *mutex)
{
    if (mutex == NULL || mutex == &gGlobalMutex) {
        initGlobalMutex();
    } else {
        umtx_lock(NULL);
        if (*mutex != NULL) {
            /* Another thread initialized this mutex first. */
            umtx_unlock(NULL);
            return;
        }

        if (pMutexInitFn != NULL) {
            UErrorCode status = U_ZERO_ERROR;
            (*pMutexInitFn)(gMutexContext, mutex, &status);
            /* TODO:  how to report failure on init? */
            umtx_unlock(NULL);
            return;
        }
        else {
#if (ICU_USE_THREADS == 1)
            /*  Search through our pool of pre-allocated mutexes for one that is not
            *  already in use.    */
            int i;
            for (i=0; i<MAX_MUTEXES; i++) {
                if (gMutexesInUse[i] == 0) {
                    gMutexesInUse[i] = 1;
                    *mutex = &gMutexes[i];
                    break;
                }
            }
#endif
        }
        umtx_unlock(NULL);

#if (ICU_USE_THREADS == 1)
        /* No more mutexes were available from our pre-allocated pool.  */
        /*   TODO:  how best to deal with this?                    */
        U_ASSERT(*mutex != NULL);
#endif
    }
}


U_CAPI void  U_EXPORT2
umtx_destroy(UMTX *mutex) {
    if (mutex == NULL) {  /* destroy the global mutex */
        mutex = &gGlobalMutex;
    }
    
    if (*mutex == NULL) {  /* someone already did it. */
        return;
    }

    /*  The life of the inc/dec mutex is tied to that of the global mutex.  */
    if (mutex == &gGlobalMutex) {
        umtx_destroy(&gIncDecMutex);
    }

    if (pMutexDestroyFn != NULL) {
        /* Mutexes are being managed by the app.  Call back to it for the destroy. */
        (*pMutexDestroyFn)(gMutexContext, mutex);
    }
    else {
#if (ICU_USE_THREADS == 1)
        /* Return this mutex to the pool of available mutexes, if it came from the
         *  pool in the first place.
         */
        /* TODO use pointer math here, instead of iterating! */
        int i;
        for (i=0; i<MAX_MUTEXES; i++)  {
            if (*mutex == &gMutexes[i]) {
                gMutexesInUse[i] = 0;
                break;
            }
        }
#endif
    }

    *mutex = NULL;
}



U_CAPI void U_EXPORT2 
u_setMutexFunctions(const void *context, UMtxInitFn *i, UMtxFn *d, UMtxFn *l, UMtxFn *u,
                    UErrorCode *status) {
    if (U_FAILURE(*status)) {
        return;
    }

    /* Can not set a mutex function to a NULL value  */
    if (i==NULL || d==NULL || l==NULL || u==NULL) {
        *status = U_ILLEGAL_ARGUMENT_ERROR;
        return;
    }

    /* If ICU is not in an initial state, disallow this operation. */
    if (cmemory_inUse()) {
        *status = U_INVALID_STATE_ERROR;
        return;
    }

    /* Swap in the mutex function pointers.  */
    pMutexInitFn    = i;
    pMutexDestroyFn = d;
    pMutexLockFn    = l;
    pMutexUnlockFn  = u;
    gMutexContext   = context;
    gGlobalMutex    = NULL;         /* For POSIX, the global mutex will be pre-initialized */
                                    /*   Undo that, force re-initialization when u_init()  */
                                    /*   happens.                                          */
}




/* Pointers to user-supplied inc/dec functions.  Null if no funcs have been set.  */
static UMtxAtomicFn  *pIncFn = NULL;
static UMtxAtomicFn  *pDecFn = NULL;
static const void *gIncDecContext  = NULL;


U_CAPI int32_t U_EXPORT2
umtx_atomic_inc(int32_t *p)  {
    int32_t retVal;
    if (pIncFn) {
        retVal = (*pIncFn)(gIncDecContext, p);
    } else {
        #if defined (U_WINDOWS) && ICU_USE_THREADS == 1
            retVal = InterlockedIncrement((LONG*)p);
        #elif defined(USE_MAC_OS_ATOMIC_INCREMENT)
            retVal = OSAtomicIncrement32Barrier(p);
        #elif defined (POSIX) && ICU_USE_THREADS == 1
            umtx_lock(&gIncDecMutex);
            retVal = ++(*p);
            umtx_unlock(&gIncDecMutex);
        #else
            /* Unknown Platform, or ICU thread support compiled out. */
            retVal = ++(*p);
        #endif
    }
    return retVal;
}

U_CAPI int32_t U_EXPORT2
umtx_atomic_dec(int32_t *p) {
    int32_t retVal;
    if (pDecFn) {
        retVal = (*pDecFn)(gIncDecContext, p);
    } else {
        #if defined (U_WINDOWS) && ICU_USE_THREADS == 1
            retVal = InterlockedDecrement((LONG*)p);
        #elif defined(USE_MAC_OS_ATOMIC_INCREMENT)
            retVal = OSAtomicDecrement32Barrier(p);
        #elif defined (POSIX) && ICU_USE_THREADS == 1
            umtx_lock(&gIncDecMutex);
            retVal = --(*p);
            umtx_unlock(&gIncDecMutex);
        #else
            /* Unknown Platform, or ICU thread support compiled out. */
            retVal = --(*p);
        #endif
    }
    return retVal;
}

/* TODO:  Some POSIXy platforms have atomic inc/dec functions available.  Use them. */





U_CAPI void U_EXPORT2
u_setAtomicIncDecFunctions(const void *context, UMtxAtomicFn *ip, UMtxAtomicFn *dp,
                                UErrorCode *status) {
    if (U_FAILURE(*status)) {
        return;
    }
    /* Can not set a mutex function to a NULL value  */
    if (ip==NULL || dp==NULL) {
        *status = U_ILLEGAL_ARGUMENT_ERROR;
        return;
    }
    /* If ICU is not in an initial state, disallow this operation. */
    if (cmemory_inUse()) {
        *status = U_INVALID_STATE_ERROR;
        return;
    }

    pIncFn = ip;
    pDecFn = dp;
    gIncDecContext = context;

#if !U_RELEASE
    {
        int32_t   testInt = 0;
        U_ASSERT(umtx_atomic_inc(&testInt) == 1);     /* Sanity Check.    Do the functions work at all? */
        U_ASSERT(testInt == 1);
        U_ASSERT(umtx_atomic_dec(&testInt) == 0);
        U_ASSERT(testInt == 0);
    }
#endif
}



U_CFUNC UBool umtx_cleanup(void) {
    umtx_destroy(NULL);
    pMutexInitFn    = NULL;
    pMutexDestroyFn = NULL;
    pMutexLockFn    = NULL;
    pMutexUnlockFn  = NULL;
    gMutexContext   = NULL;
    gGlobalMutex    = NULL;
    pIncFn          = NULL;
    pDecFn          = NULL;
    gIncDecContext  = NULL;
    gIncDecMutex    = NULL;

#if (ICU_USE_THREADS == 1)
    if (gMutexPoolInitialized) {
        int i;
        for (i=0; i<MAX_MUTEXES; i++) {
            if (gMutexesInUse[i]) {
#if defined (U_WINDOWS)
                DeleteCriticalSection(&gMutexes[i]);
#elif defined (POSIX)
                pthread_mutex_destroy(&gMutexes[i]);
#endif
                gMutexesInUse[i] = 0;
            }
        }
    }
    gMutexPoolInitialized = FALSE;
#endif

    return TRUE;
}


