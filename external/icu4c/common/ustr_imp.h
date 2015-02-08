
#ifndef __USTR_IMP_H__
#define __USTR_IMP_H__

#include "unicode/utypes.h"
#include "unicode/uiter.h"
#include "ucase.h"

/** Simple declaration for u_strToTitle() to avoid including unicode/ubrk.h. */
#ifndef UBRK_TYPEDEF_UBREAK_ITERATOR
#   define UBRK_TYPEDEF_UBREAK_ITERATOR
    typedef struct UBreakIterator UBreakIterator;
#endif

#ifndef U_COMPARE_IGNORE_CASE
/* see also unorm.h */
#define U_COMPARE_IGNORE_CASE       0x10000
#endif

#define _STRNCMP_STYLE 0x1000

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
