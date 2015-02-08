
#ifndef __UARRSORT_H__
#define __UARRSORT_H__

#include "unicode/utypes.h"

U_CDECL_BEGIN
typedef int32_t U_CALLCONV
UComparator(const void *context, const void *left, const void *right);
U_CDECL_END

U_CAPI void U_EXPORT2
uprv_sortArray(void *array, int32_t length, int32_t itemSize,
               UComparator *cmp, const void *context,
               UBool sortStable, UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
uprv_uint16Comparator(const void *context, const void *left, const void *right);

U_CAPI int32_t U_EXPORT2
uprv_int32Comparator(const void *context, const void *left, const void *right);

U_CAPI int32_t U_EXPORT2
uprv_uint32Comparator(const void *context, const void *left, const void *right);

#endif
