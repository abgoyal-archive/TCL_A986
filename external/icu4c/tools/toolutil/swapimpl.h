
#ifndef __SWAPIMPL_H__
#define __SWAPIMPL_H__

#include "unicode/utypes.h"
#include "udataswp.h"

U_CAPI int32_t U_EXPORT2
udata_swap(const UDataSwapper *ds,
           const void *inData, int32_t length, void *outData,
           UErrorCode *pErrorCode);

#endif
