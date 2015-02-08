
#ifndef UCOL_WGT_H
#define UCOL_WGT_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_COLLATION

/* definitions for CE weights */

typedef struct WeightRange {
    uint32_t start, end;
    int32_t length, count;
    int32_t length2;
    uint32_t count2;
} WeightRange;

U_CFUNC int32_t
ucol_allocWeights(uint32_t lowerLimit, uint32_t upperLimit,
                  uint32_t n,
                  uint32_t maxByte,
                  WeightRange ranges[7]);

U_CFUNC uint32_t
ucol_nextWeight(WeightRange ranges[], int32_t *pRangeCount);

#endif /* #if !UCONFIG_NO_COLLATION */

#endif
