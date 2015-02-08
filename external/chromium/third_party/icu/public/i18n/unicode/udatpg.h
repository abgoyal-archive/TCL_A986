
#ifndef __UDATPG_H__
#define __UDATPG_H__

#include "unicode/utypes.h"
#include "unicode/uenum.h"


typedef void *UDateTimePatternGenerator;

typedef enum UDateTimePatternField {
    /** @stable ICU 3.8 */
    UDATPG_ERA_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_YEAR_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_QUARTER_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_MONTH_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_WEEK_OF_YEAR_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_WEEK_OF_MONTH_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_WEEKDAY_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_DAY_OF_YEAR_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_DAY_OF_WEEK_IN_MONTH_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_DAY_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_DAYPERIOD_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_HOUR_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_MINUTE_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_SECOND_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_FRACTIONAL_SECOND_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_ZONE_FIELD,
    /** @stable ICU 3.8 */
    UDATPG_FIELD_COUNT
} UDateTimePatternField;

typedef enum UDateTimePatternConflict {
    /** @stable ICU 3.8 */
    UDATPG_NO_CONFLICT,
    /** @stable ICU 3.8 */
    UDATPG_BASE_CONFLICT,
    /** @stable ICU 3.8 */
    UDATPG_CONFLICT,
    /** @stable ICU 3.8 */
    UDATPG_CONFLICT_COUNT
} UDateTimePatternConflict;

U_DRAFT UDateTimePatternGenerator * U_EXPORT2
udatpg_open(const char *locale, UErrorCode *pErrorCode);

U_DRAFT UDateTimePatternGenerator * U_EXPORT2
udatpg_openEmpty(UErrorCode *pErrorCode);

U_DRAFT void U_EXPORT2
udatpg_close(UDateTimePatternGenerator *dtpg);

U_DRAFT UDateTimePatternGenerator * U_EXPORT2
udatpg_clone(const UDateTimePatternGenerator *dtpg, UErrorCode *pErrorCode);

U_DRAFT int32_t U_EXPORT2
udatpg_getBestPattern(UDateTimePatternGenerator *dtpg,
                      const UChar *skeleton, int32_t length,
                      UChar *bestPattern, int32_t capacity,
                      UErrorCode *pErrorCode);

U_DRAFT int32_t U_EXPORT2
udatpg_getSkeleton(UDateTimePatternGenerator *dtpg,
                   const UChar *pattern, int32_t length,
                   UChar *skeleton, int32_t capacity,
                   UErrorCode *pErrorCode);

U_DRAFT int32_t U_EXPORT2
udatpg_getBaseSkeleton(UDateTimePatternGenerator *dtpg,
                       const UChar *pattern, int32_t length,
                       UChar *baseSkeleton, int32_t capacity,
                       UErrorCode *pErrorCode);

U_DRAFT UDateTimePatternConflict U_EXPORT2
udatpg_addPattern(UDateTimePatternGenerator *dtpg,
                  const UChar *pattern, int32_t patternLength,
                  UBool override,
                  UChar *conflictingPattern, int32_t capacity, int32_t *pLength,
                  UErrorCode *pErrorCode);

U_DRAFT void U_EXPORT2
udatpg_setAppendItemFormat(UDateTimePatternGenerator *dtpg,
                           UDateTimePatternField field,
                           const UChar *value, int32_t length);

U_DRAFT const UChar * U_EXPORT2
udatpg_getAppendItemFormat(const UDateTimePatternGenerator *dtpg,
                           UDateTimePatternField field,
                           int32_t *pLength);

U_DRAFT void U_EXPORT2
udatpg_setAppendItemName(UDateTimePatternGenerator *dtpg,
                         UDateTimePatternField field,
                         const UChar *value, int32_t length);

U_DRAFT const UChar * U_EXPORT2
udatpg_getAppendItemName(const UDateTimePatternGenerator *dtpg,
                         UDateTimePatternField field,
                         int32_t *pLength);

U_DRAFT void U_EXPORT2
udatpg_setDateTimeFormat(const UDateTimePatternGenerator *dtpg,
                         const UChar *dtFormat, int32_t length);

U_DRAFT const UChar * U_EXPORT2
udatpg_getDateTimeFormat(const UDateTimePatternGenerator *dtpg,
                         int32_t *pLength);

U_DRAFT void U_EXPORT2
udatpg_setDecimal(UDateTimePatternGenerator *dtpg,
                  const UChar *decimal, int32_t length);

U_DRAFT const UChar * U_EXPORT2
udatpg_getDecimal(const UDateTimePatternGenerator *dtpg,
                  int32_t *pLength);

U_DRAFT int32_t U_EXPORT2
udatpg_replaceFieldTypes(UDateTimePatternGenerator *dtpg,
                         const UChar *pattern, int32_t patternLength,
                         const UChar *skeleton, int32_t skeletonLength,
                         UChar *dest, int32_t destCapacity,
                         UErrorCode *pErrorCode);

U_DRAFT UEnumeration * U_EXPORT2
udatpg_openSkeletons(const UDateTimePatternGenerator *dtpg, UErrorCode *pErrorCode);

U_DRAFT UEnumeration * U_EXPORT2
udatpg_openBaseSkeletons(const UDateTimePatternGenerator *dtpg, UErrorCode *pErrorCode);

U_DRAFT const UChar * U_EXPORT2
udatpg_getPatternForSkeleton(const UDateTimePatternGenerator *dtpg,
                             const UChar *skeleton, int32_t skeletonLength,
                             int32_t *pLength);

#endif
