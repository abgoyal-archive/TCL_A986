
#ifndef ULOCIMP_H
#define ULOCIMP_H

#include "unicode/uloc.h"

U_CAPI UEnumeration* U_EXPORT2
uloc_openKeywordList(const char *keywordList, int32_t keywordListSize, UErrorCode* status);

#endif
