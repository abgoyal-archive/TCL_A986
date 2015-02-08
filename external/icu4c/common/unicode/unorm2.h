
#ifndef __UNORM2_H__
#define __UNORM2_H__


#include "unicode/utypes.h"
#include "unicode/localpointer.h"
#include "unicode/uset.h"

typedef enum {
    /**
     * Decomposition followed by composition.
     * Same as standard NFC when using an "nfc" instance.
     * Same as standard NFKC when using an "nfkc" instance.
     * For details about standard Unicode normalization forms
     * see http://www.unicode.org/unicode/reports/tr15/
     * @draft ICU 4.4
     */
    UNORM2_COMPOSE,
    /**
     * Map, and reorder canonically.
     * Same as standard NFD when using an "nfc" instance.
     * Same as standard NFKD when using an "nfkc" instance.
     * For details about standard Unicode normalization forms
     * see http://www.unicode.org/unicode/reports/tr15/
     * @draft ICU 4.4
     */
    UNORM2_DECOMPOSE,
    /**
     * "Fast C or D" form.
     * If a string is in this form, then further decomposition <i>without reordering</i>
     * would yield the same form as DECOMPOSE.
     * Text in "Fast C or D" form can be processed efficiently with data tables
     * that are "canonically closed", that is, that provide equivalent data for
     * equivalent text, without having to be fully normalized.
     * Not a standard Unicode normalization form.
     * Not a unique form: Different FCD strings can be canonically equivalent.
     * For details see http://www.unicode.org/notes/tn5/#FCD
     * @draft ICU 4.4
     */
    UNORM2_FCD,
    /**
     * Compose only contiguously.
     * Also known as "FCC" or "Fast C Contiguous".
     * The result will often but not always be in NFC.
     * The result will conform to FCD which is useful for processing.
     * Not a standard Unicode normalization form.
     * For details see http://www.unicode.org/notes/tn5/#FCC
     * @draft ICU 4.4
     */
    UNORM2_COMPOSE_CONTIGUOUS
} UNormalization2Mode;

typedef enum UNormalizationCheckResult {
  /** 
   * The input string is not in the normalization form.
   * @stable ICU 2.0
   */
  UNORM_NO,
  /** 
   * The input string is in the normalization form.
   * @stable ICU 2.0
   */
  UNORM_YES,
  /** 
   * The input string may or may not be in the normalization form.
   * This value is only returned for composition forms like NFC and FCC,
   * when a backward-combining character is found for which the surrounding text
   * would have to be analyzed further.
   * @stable ICU 2.0
   */
  UNORM_MAYBE
} UNormalizationCheckResult;

struct UNormalizer2;
typedef struct UNormalizer2 UNormalizer2;  /**< C typedef for struct UNormalizer2. @draft ICU 4.4 */

#if !UCONFIG_NO_NORMALIZATION

U_DRAFT const UNormalizer2 * U_EXPORT2
unorm2_getInstance(const char *packageName,
                   const char *name,
                   UNormalization2Mode mode,
                   UErrorCode *pErrorCode);

U_DRAFT UNormalizer2 * U_EXPORT2
unorm2_openFiltered(const UNormalizer2 *norm2, const USet *filterSet, UErrorCode *pErrorCode);

U_DRAFT void U_EXPORT2
unorm2_close(UNormalizer2 *norm2);

#if U_SHOW_CPLUSPLUS_API

U_NAMESPACE_BEGIN

U_DEFINE_LOCAL_OPEN_POINTER(LocalUNormalizer2Pointer, UNormalizer2, unorm2_close);

U_NAMESPACE_END

#endif

U_DRAFT int32_t U_EXPORT2
unorm2_normalize(const UNormalizer2 *norm2,
                 const UChar *src, int32_t length,
                 UChar *dest, int32_t capacity,
                 UErrorCode *pErrorCode);
U_DRAFT int32_t U_EXPORT2
unorm2_normalizeSecondAndAppend(const UNormalizer2 *norm2,
                                UChar *first, int32_t firstLength, int32_t firstCapacity,
                                const UChar *second, int32_t secondLength,
                                UErrorCode *pErrorCode);
U_DRAFT int32_t U_EXPORT2
unorm2_append(const UNormalizer2 *norm2,
              UChar *first, int32_t firstLength, int32_t firstCapacity,
              const UChar *second, int32_t secondLength,
              UErrorCode *pErrorCode);

U_DRAFT UBool U_EXPORT2
unorm2_isNormalized(const UNormalizer2 *norm2,
                    const UChar *s, int32_t length,
                    UErrorCode *pErrorCode);

U_DRAFT UNormalizationCheckResult U_EXPORT2
unorm2_quickCheck(const UNormalizer2 *norm2,
                  const UChar *s, int32_t length,
                  UErrorCode *pErrorCode);

U_DRAFT int32_t U_EXPORT2
unorm2_spanQuickCheckYes(const UNormalizer2 *norm2,
                         const UChar *s, int32_t length,
                         UErrorCode *pErrorCode);

U_DRAFT UBool U_EXPORT2
unorm2_hasBoundaryBefore(const UNormalizer2 *norm2, UChar32 c);

U_DRAFT UBool U_EXPORT2
unorm2_hasBoundaryAfter(const UNormalizer2 *norm2, UChar32 c);

U_DRAFT UBool U_EXPORT2
unorm2_isInert(const UNormalizer2 *norm2, UChar32 c);

#endif  /* !UCONFIG_NO_NORMALIZATION */
#endif  /* __UNORM2_H__ */
