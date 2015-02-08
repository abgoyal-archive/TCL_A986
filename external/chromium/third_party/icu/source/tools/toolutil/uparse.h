
#ifndef __UPARSE_H__
#define __UPARSE_H__

#include "unicode/utypes.h"

U_CDECL_BEGIN

U_CAPI const char * U_EXPORT2
u_skipWhitespace(const char *s);

/** Function type for u_parseDelimitedFile(). */
typedef void U_CALLCONV
UParseLineFn(void *context,
              char *fields[][2],
              int32_t fieldCount,
              UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
u_parseDelimitedFile(const char *filename, char delimiter,
                     char *fields[][2], int32_t fieldCount,
                     UParseLineFn *lineFn, void *context,
                     UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_parseCodePoints(const char *s,
                  uint32_t *dest, int32_t destCapacity,
                  UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_parseString(const char *s,
              UChar *dest, int32_t destCapacity,
              uint32_t *pFirst,
              UErrorCode *pErrorCode);

U_CAPI int32_t U_EXPORT2
u_parseCodePointRange(const char *s,
                      uint32_t *pStart, uint32_t *pEnd,
                      UErrorCode *pErrorCode);


U_CAPI int32_t U_EXPORT2
u_parseUTF8(const char *source, int32_t sLen, char *dest, int32_t destCapacity, UErrorCode *status);

U_CDECL_END

#endif
