
#ifndef __USTR_IMP_H__
#define __USTR_IMP_H__

#include "unicode/utypes.h"
#include "unicode/uiter.h"
#include "ucase.h"

/** Simple declaration for u_strToTitle() to avoid including unicode/ubrk.h. */
#ifndef UBRK_TYPEDEF_UBREAK_ITERATOR
#   define UBRK_TYPEDEF_UBREAK_ITERATOR
    typedef void UBreakIterator;
#endif

U_CFUNC int32_t U_EXPORT2
uprv_strCompare(const UChar *s1, int32_t length1,
                const UChar *s2, int32_t length2,
                UBool strncmpStyle, UBool codePointOrder);

U_CFUNC int32_t
u_strcmpFold(const UChar *s1, int32_t length1,
             const UChar *s2, int32_t length2,
             uint32_t options,
             UErrorCode *pErrorCode);

U_CFUNC UBool
uprv_haveProperties(UErrorCode *pErrorCode);


typedef UBool U_CALLCONV
UGrowBuffer(void *context,      /* opaque pointer for this function */
            UChar **pBuffer,    /* in/out destination buffer pointer */
            int32_t *pCapacity, /* in/out buffer capacity in numbers of UChars */
            int32_t reqCapacity,/* requested capacity */
            int32_t length);    /* number of UChars to be copied to new buffer */

U_CAPI UBool /* U_CALLCONV U_EXPORT2 */
u_growBufferFromStatic(void *context,
                       UChar **pBuffer, int32_t *pCapacity, int32_t reqCapacity,
                       int32_t length);


struct UCaseMap {
    const UCaseProps *csp;
#if !UCONFIG_NO_BREAK_ITERATION
    UBreakIterator *iter;  /* We adopt the iterator, so we own it. */
#endif
    char locale[32];
    int32_t locCache;
    uint32_t options;
};

#ifndef __UCASEMAP_H__
typedef struct UCaseMap UCaseMap;
#endif

enum {
    TO_LOWER,
    TO_UPPER,
    TO_TITLE,
    FOLD_CASE
};

U_CFUNC int32_t
ustr_toLower(const UCaseProps *csp,
             UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             const char *locale,
             UErrorCode *pErrorCode);

U_CFUNC int32_t
ustr_toUpper(const UCaseProps *csp,
             UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             const char *locale,
             UErrorCode *pErrorCode);

#if !UCONFIG_NO_BREAK_ITERATION

U_CFUNC int32_t
ustr_toTitle(const UCaseProps *csp,
             UChar *dest, int32_t destCapacity,
             const UChar *src, int32_t srcLength,
             UBreakIterator *titleIter,
             const char *locale, uint32_t options,
             UErrorCode *pErrorCode);

#endif

U_CFUNC int32_t
ustr_foldCase(const UCaseProps *csp,
              UChar *dest, int32_t destCapacity,
              const UChar *src, int32_t srcLength,
              uint32_t options,
              UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_terminateUChars(UChar *dest, int32_t destCapacity, int32_t length, UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_terminateChars(char *dest, int32_t destCapacity, int32_t length, UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_terminateUChar32s(UChar32 *dest, int32_t destCapacity, int32_t length, UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_terminateWChars(wchar_t *dest, int32_t destCapacity, int32_t length, UErrorCode *pErrorCode);

#endif
