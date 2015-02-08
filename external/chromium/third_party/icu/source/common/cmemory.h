
#ifndef CMEMORY_H
#define CMEMORY_H

#include "unicode/utypes.h"
#include <stddef.h>
#include <string.h>


#define uprv_memcpy(dst, src, size) U_STANDARD_CPP_NAMESPACE memcpy(dst, src, size)
#define uprv_memmove(dst, src, size) U_STANDARD_CPP_NAMESPACE memmove(dst, src, size)
#define uprv_memset(buffer, mark, size) U_STANDARD_CPP_NAMESPACE memset(buffer, mark, size)
#define uprv_memcmp(buffer1, buffer2, size) U_STANDARD_CPP_NAMESPACE memcmp(buffer1, buffer2,size)

U_CAPI void * U_EXPORT2
uprv_malloc(size_t s);

U_CAPI void * U_EXPORT2
uprv_realloc(void *mem, size_t size);

U_CAPI void U_EXPORT2
uprv_free(void *mem);

typedef union {
    long    t1;
    double  t2;
    void   *t3;
} UAlignedMemory;

#define U_POINTER_MASK_LSB(ptr, mask) (((ptrdiff_t)(char *)(ptr)) & (mask))

#define U_ALIGNMENT_OFFSET(ptr) U_POINTER_MASK_LSB(ptr, sizeof(UAlignedMemory) - 1)

#define U_ALIGNMENT_OFFSET_UP(ptr) (sizeof(UAlignedMemory) - U_ALIGNMENT_OFFSET(ptr))

U_CFUNC UBool 
cmemory_inUse(void);

U_CFUNC UBool 
cmemory_cleanup(void);

#endif
