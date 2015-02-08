
#ifndef UNORM_H
#define UNORM_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_NORMALIZATION

#include "unicode/uiter.h"


typedef enum {
  /** No decomposition/composition. @stable ICU 2.0 */
  UNORM_NONE = 1, 
  /** Canonical decomposition. @stable ICU 2.0 */
  UNORM_NFD = 2,
  /** Compatibility decomposition. @stable ICU 2.0 */
  UNORM_NFKD = 3,
  /** Canonical decomposition followed by canonical composition. @stable ICU 2.0 */
  UNORM_NFC = 4,
  /** Default normalization. @stable ICU 2.0 */
  UNORM_DEFAULT = UNORM_NFC, 
  /** Compatibility decomposition followed by canonical composition. @stable ICU 2.0 */
  UNORM_NFKC =5,
  /** "Fast C or D" form. @stable ICU 2.0 */
  UNORM_FCD = 6,

  /** One more than the highest normalization mode constant. @stable ICU 2.0 */
  UNORM_MODE_COUNT
} UNormalizationMode;

enum {
    /**
     * Options bit set value to select Unicode 3.2 normalization
     * (except NormalizationCorrections).
     * At most one Unicode version can be selected at a time.
     * @stable ICU 2.6
     */
    UNORM_UNICODE_3_2=0x20
};

#define UNORM_COMPARE_NORM_OPTIONS_SHIFT 20

U_STABLE int32_t U_EXPORT2 
unorm_normalize(const UChar *source, int32_t sourceLength,
                UNormalizationMode mode, int32_t options,
                UChar *result, int32_t resultLength,
                UErrorCode *status);
#endif
typedef enum UNormalizationCheckResult {
  /** 
   * Indicates that string is not in the normalized format
   */
  UNORM_NO,
  /** 
   * Indicates that string is in the normalized format
   */
  UNORM_YES,
  /** 
   * Indicates that string cannot be determined if it is in the normalized 
   * format without further thorough checks.
   */
  UNORM_MAYBE
} UNormalizationCheckResult;
#if !UCONFIG_NO_NORMALIZATION
U_STABLE UNormalizationCheckResult U_EXPORT2
unorm_quickCheck(const UChar *source, int32_t sourcelength,
                 UNormalizationMode mode,
                 UErrorCode *status);

U_STABLE UNormalizationCheckResult U_EXPORT2
unorm_quickCheckWithOptions(const UChar *src, int32_t srcLength, 
                            UNormalizationMode mode, int32_t options,
                            UErrorCode *pErrorCode);

U_STABLE UBool U_EXPORT2
unorm_isNormalized(const UChar *src, int32_t srcLength,
                   UNormalizationMode mode,
                   UErrorCode *pErrorCode);

U_STABLE UBool U_EXPORT2
unorm_isNormalizedWithOptions(const UChar *src, int32_t srcLength,
                              UNormalizationMode mode, int32_t options,
                              UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
unorm_next(UCharIterator *src,
           UChar *dest, int32_t destCapacity,
           UNormalizationMode mode, int32_t options,
           UBool doNormalize, UBool *pNeededToNormalize,
           UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
unorm_previous(UCharIterator *src,
               UChar *dest, int32_t destCapacity,
               UNormalizationMode mode, int32_t options,
               UBool doNormalize, UBool *pNeededToNormalize,
               UErrorCode *pErrorCode);

U_STABLE int32_t U_EXPORT2
unorm_concatenate(const UChar *left, int32_t leftLength,
                  const UChar *right, int32_t rightLength,
                  UChar *dest, int32_t destCapacity,
                  UNormalizationMode mode, int32_t options,
                  UErrorCode *pErrorCode);

#define UNORM_INPUT_IS_FCD          0x20000

#define U_COMPARE_IGNORE_CASE       0x10000

#ifndef U_COMPARE_CODE_POINT_ORDER
/* see also unistr.h and ustring.h */
#define U_COMPARE_CODE_POINT_ORDER  0x8000
#endif

U_STABLE int32_t U_EXPORT2
unorm_compare(const UChar *s1, int32_t length1,
              const UChar *s2, int32_t length2,
              uint32_t options,
              UErrorCode *pErrorCode);

#endif /* #if !UCONFIG_NO_NORMALIZATION */

#endif
