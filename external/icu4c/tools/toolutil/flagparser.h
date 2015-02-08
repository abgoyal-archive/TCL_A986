
#ifndef __FLAGPARSER_H__
#define __FLAGPARSER_H__

#include "unicode/utypes.h"

U_CAPI void U_EXPORT2
parseFlagsFile(const char *fileName, char **flagBuffer, int32_t flagBufferSize, int32_t numOfFlags, UErrorCode *status);

#endif
