

#ifndef _BMS_H
#define _BMS_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

#include "unicode/ucol.h"

typedef void UCD;

U_CAPI UCD * U_EXPORT2
ucd_open(UCollator *coll, UErrorCode *status);

U_CAPI void U_EXPORT2
ucd_close(UCD *ucd);

U_CAPI UCollator * U_EXPORT2
ucd_getCollator(UCD *ucd);

U_CAPI void U_EXPORT2
ucd_freeCache();

U_CAPI void U_EXPORT2
ucd_flushCache();

struct BMS;
typedef struct BMS BMS; /**< @see BMS */

U_CAPI BMS * U_EXPORT2
bms_open(UCD *ucd,
         const UChar *pattern, int32_t patternLength,
         const UChar *target,  int32_t targetLength,
         UErrorCode  *status);

U_CAPI void U_EXPORT2
bms_close(BMS *bms);

U_CAPI UBool U_EXPORT2
bms_empty(BMS *bms);

U_CAPI UCD * U_EXPORT2
bms_getData(BMS *bms);

U_CAPI UBool U_EXPORT2
bms_search(BMS *bms, int32_t offset, int32_t *start, int32_t *end);

U_CAPI void U_EXPORT2
bms_setTargetString(BMS *bms, const UChar *target, int32_t targetLength, UErrorCode *status);

#endif

#endif /* _BMS_H */
