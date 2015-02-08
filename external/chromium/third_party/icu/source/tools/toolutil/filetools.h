
#ifndef __FILETOOLS_H__
#define __FILETOOLS_H__

#include "unicode/utypes.h"

U_CAPI UBool U_EXPORT2
isFileModTimeLater(const char *filePath, const char *checkAgainst, UBool isDir=FALSE);

U_CAPI void U_EXPORT2
swapFileSepChar(char *filePath, const char oldFileSepChar, const char newFileSepChar);

#endif
