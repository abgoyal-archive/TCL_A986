

#ifndef PUTIL_H
#define PUTIL_H

#include "unicode/utypes.h"

#ifndef IEEE_754
#   define IEEE_754 1
#endif

/*==========================================================================*/
/* Platform utilities                                                       */
/*==========================================================================*/


U_STABLE const char* U_EXPORT2 u_getDataDirectory(void);

U_STABLE void U_EXPORT2 u_setDataDirectory(const char *directory);

U_INTERNAL const char*  U_EXPORT2 uprv_getDefaultCodepage(void);

U_INTERNAL const char*  U_EXPORT2 uprv_getDefaultLocaleID(void);

#ifdef XP_MAC
#   define U_FILE_SEP_CHAR ':'
#   define U_FILE_ALT_SEP_CHAR ':'
#   define U_PATH_SEP_CHAR ';'
#   define U_FILE_SEP_STRING ":"
#   define U_FILE_ALT_SEP_STRING ":"
#   define U_PATH_SEP_STRING ";"
#elif defined(WIN32) || defined(OS2)
#   define U_FILE_SEP_CHAR '\\'
#   define U_FILE_ALT_SEP_CHAR '/'
#   define U_PATH_SEP_CHAR ';'
#   define U_FILE_SEP_STRING "\\"
#   define U_FILE_ALT_SEP_STRING "/"
#   define U_PATH_SEP_STRING ";"
#else
#   define U_FILE_SEP_CHAR '/'
#   define U_FILE_ALT_SEP_CHAR '/'
#   define U_PATH_SEP_CHAR ':'
#   define U_FILE_SEP_STRING "/"
#   define U_FILE_ALT_SEP_STRING "/"
#   define U_PATH_SEP_STRING ":"
#endif

U_STABLE void U_EXPORT2
u_charsToUChars(const char *cs, UChar *us, int32_t length);

U_STABLE void U_EXPORT2
u_UCharsToChars(const UChar *us, char *cs, int32_t length);

#endif
