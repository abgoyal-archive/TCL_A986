
#ifndef __ULOCDATA_H__
#define __ULOCDATA_H__

#include "unicode/ures.h"
#include "unicode/uloc.h"
#include "unicode/uset.h"


/** Forward declaration of the ULocaleData structure. @stable ICU 3.6 */
struct ULocaleData;

/** A locale data object. @stable ICU 3.6 */
typedef struct ULocaleData ULocaleData;



typedef enum ULocaleDataExemplarSetType  {
     ULOCDATA_ES_STANDARD=0,      /* Basic set */
     ULOCDATA_ES_AUXILIARY=1,     /* Auxiliary set */
     ULOCDATA_ES_COUNT=2
} ULocaleDataExemplarSetType;

typedef enum ULocaleDataDelimiterType {
    ULOCDATA_QUOTATION_START = 0,     /* Quotation start */
     ULOCDATA_QUOTATION_END = 1,       /* Quotation end */
     ULOCDATA_ALT_QUOTATION_START = 2, /* Alternate quotation start */
     ULOCDATA_ALT_QUOTATION_END = 3,   /* Alternate quotation end */
     ULOCDATA_DELIMITER_COUNT = 4
} ULocaleDataDelimiterType;

U_STABLE ULocaleData* U_EXPORT2
ulocdata_open(const char *localeID, UErrorCode *status);

U_STABLE void U_EXPORT2
ulocdata_close(ULocaleData *uld);

U_STABLE void U_EXPORT2
ulocdata_setNoSubstitute(ULocaleData *uld, UBool setting);

U_STABLE UBool U_EXPORT2
ulocdata_getNoSubstitute(ULocaleData *uld);

U_STABLE USet* U_EXPORT2
ulocdata_getExemplarSet(ULocaleData *uld, USet *fillIn,
                        uint32_t options, ULocaleDataExemplarSetType extype, UErrorCode *status);

U_STABLE int32_t U_EXPORT2
ulocdata_getDelimiter(ULocaleData *uld, ULocaleDataDelimiterType type, UChar *result, int32_t resultLength, UErrorCode *status);

typedef enum UMeasurementSystem {
    UMS_SI,     /** Measurement system specified by SI otherwise known as Metric system. */
    UMS_US,     /** Measurement system followed in the United States of America. */
    UMS_LIMIT
} UMeasurementSystem;

U_STABLE UMeasurementSystem U_EXPORT2
ulocdata_getMeasurementSystem(const char *localeID, UErrorCode *status);

U_STABLE void U_EXPORT2
ulocdata_getPaperSize(const char *localeID, int32_t *height, int32_t *width, UErrorCode *status);

U_DRAFT void U_EXPORT2
ulocdata_getCLDRVersion(UVersionInfo versionArray, UErrorCode *status);

U_DRAFT int32_t U_EXPORT2
ulocdata_getLocaleDisplayPattern(ULocaleData *uld,
                                 UChar *pattern,
                                 int32_t patternCapacity,
                                 UErrorCode *status);


U_DRAFT int32_t U_EXPORT2
ulocdata_getLocaleSeparator(ULocaleData *uld,
                            UChar *separator,
                            int32_t separatorCapacity,
                            UErrorCode *status);
#endif
