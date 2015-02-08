
#ifndef __UNORM_IT_H__
#define __UNORM_IT_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION && !UCONFIG_NO_NORMALIZATION

#include "unicode/uiter.h"
#include "unicode/unorm.h"

struct UNormIterator;
typedef struct UNormIterator UNormIterator;

#define UNORM_ITER_SIZE 1024

U_CAPI UNormIterator * U_EXPORT2
unorm_openIter(void *stackMem, int32_t stackMemSize, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
unorm_closeIter(UNormIterator *uni);

U_CAPI UCharIterator * U_EXPORT2
unorm_setIter(UNormIterator *uni, UCharIterator *iter, UNormalizationMode mode, UErrorCode *pErrorCode);

#endif /* uconfig.h switches */

#endif
