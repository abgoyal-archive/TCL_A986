
#ifndef __UCASEMAP_H__
#define __UCASEMAP_H__

#include "unicode/utypes.h"
#include "unicode/ustring.h"
#include "unicode/localpointer.h"


struct UCaseMap;
typedef struct UCaseMap UCaseMap; /**< C typedef for struct UCaseMap. @stable ICU 3.4 */

U_STABLE UCaseMap * U_EXPORT2
ucasemap_open(const char *locale, uint32_t options, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ucasemap_close(UCaseMap *csm);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUCaseMapPointer, UCaseMap, ucasemap_close);

U_NAMESPACE_END

#endif

U_STABLE const char * U_EXPORT2
ucasemap_getLocale(const UCaseMap *csm);

U_STABLE uint32_t U_EXPORT2
ucasemap_getOptions(const UCaseMap *csm);

U_STABLE void U_EXPORT2
ucasemap_setLocale(UCaseMap *csm, const char *locale, UErrorCode *pErrorCode);

U_STABLE void U_EXPORT2
ucasemap_setOptions(UCaseMap *csm, uint32_t options, UErrorCode *pErrorCode);

#define U_TITLECASE_NO_LOWERCASE 0x100

#define U_TITLECASE_NO_BREAK_ADJUSTMENT 0x200

#if !UCONFIG_NO_BREAK_ITERATION

U_STABLE const UBreakIterator * U_EXPORT2
ucasemap_getBreakIterator(const UCaseMap *csm);

U_STABLE void U_EXPORT2
ucasemap_setBreakIterator(UCaseMap *csm, UBreakIterator *iterToAdopt, UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
ucasemap_toTitle(UCaseMap *csm,
                 UChar *dest, int32_t destCapacity,
                 const UChar *src, int32_t srcLength,
                 UErrorCode *pErrorCode);

#endif

U_STABLE int32_t U_EXPORT2
ucasemap_utf8ToLower(const UCaseMap *csm,
                     char *dest, int32_t destCapacity,
                     const char *src, int32_t srcLength,
                     UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
ucasemap_utf8ToUpper(const UCaseMap *csm,
                     char *dest, int32_t destCapacity,
                     const char *src, int32_t srcLength,
                     UErrorCode *pErrorCode);

#if !UCONFIG_NO_BREAK_ITERATION

U_STABLE int32_t U_EXPORT2
ucasemap_utf8ToTitle(UCaseMap *csm,
                    char *dest, int32_t destCapacity,
                    const char *src, int32_t srcLength,
                    UErrorCode *pErrorCode);

#endif

U_STABLE int32_t U_EXPORT2
ucasemap_utf8FoldCase(const UCaseMap *csm,
                      char *dest, int32_t destCapacity,
                      const char *src, int32_t srcLength,
                      UErrorCode *pErrorCode);

#endif
