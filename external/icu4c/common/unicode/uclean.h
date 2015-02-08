
#ifndef __UCLEAN_H__
#define __UCLEAN_H__

#include "unicode/utypes.h"
 
U_STABLE void U_EXPORT2 
u_init(UErrorCode *status);

U_STABLE void U_EXPORT2 
u_cleanup(void);




typedef void *UMTX;

typedef void U_CALLCONV UMtxInitFn (const void *context, UMTX  *mutex, UErrorCode* status);


typedef void U_CALLCONV UMtxFn   (const void *context, UMTX  *mutex);


U_STABLE void U_EXPORT2 
u_setMutexFunctions(const void *context, UMtxInitFn *init, UMtxFn *destroy, UMtxFn *lock, UMtxFn *unlock,
                    UErrorCode *status);


typedef int32_t U_CALLCONV UMtxAtomicFn(const void *context, int32_t *p);

U_STABLE void U_EXPORT2 
u_setAtomicIncDecFunctions(const void *context, UMtxAtomicFn *inc, UMtxAtomicFn *dec,
                    UErrorCode *status);



typedef void *U_CALLCONV UMemAllocFn(const void *context, size_t size);
typedef void *U_CALLCONV UMemReallocFn(const void *context, void *mem, size_t size);
typedef void  U_CALLCONV UMemFreeFn (const void *context, void *mem);

U_STABLE void U_EXPORT2 
u_setMemoryFunctions(const void *context, UMemAllocFn *a, UMemReallocFn *r, UMemFreeFn *f, 
                    UErrorCode *status);

#endif
