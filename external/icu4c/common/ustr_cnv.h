
#ifndef USTR_CNV_IMP_H
#define USTR_CNV_IMP_H

#include "unicode/utypes.h"
#include "unicode/ucnv.h"

#if !UCONFIG_NO_CONVERSION

U_CAPI UConverter* U_EXPORT2
u_getDefaultConverter(UErrorCode *status);


U_CAPI void U_EXPORT2
u_releaseDefaultConverter(UConverter *converter);

#endif

#endif
