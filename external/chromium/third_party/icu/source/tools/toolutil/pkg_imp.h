
#ifndef __PKG_IMP_H__
#define __PKG_IMP_H__

#include "unicode/utypes.h"

U_CFUNC const UDataInfo *
getDataInfo(const uint8_t *data, int32_t length,
            int32_t &infoLength, int32_t &headerLength,
            UErrorCode *pErrorCode);

#endif
