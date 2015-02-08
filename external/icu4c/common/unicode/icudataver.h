
#ifndef __ICU_DATA_VER_H__
#define __ICU_DATA_VER_H__

#include "unicode/utypes.h"

#define U_ICU_VERSION_BUNDLE "icuver"

#define U_ICU_STD_BUNDLE "icustd"

#define U_ICU_DATA_KEY "DataVersion"

U_INTERNAL UBool U_EXPORT2 u_isDataOlder(UVersionInfo dataVersionFillin, UBool *isModifiedFillin, UErrorCode *status);

U_INTERNAL void U_EXPORT2 u_getDataVersion(UVersionInfo dataVersionFillin, UErrorCode *status);

#endif
