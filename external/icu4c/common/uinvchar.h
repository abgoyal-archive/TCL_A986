
#ifndef __UINVCHAR_H__
#define __UINVCHAR_H__

#include "unicode/utypes.h"

U_INTERNAL UBool U_EXPORT2
uprv_isInvariantString(const char *s, int32_t length);

U_INTERNAL UBool U_EXPORT2
uprv_isInvariantUString(const UChar *s, int32_t length);

#if U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define U_UPPER_ORDINAL(x) ((x)-'A')
#elif U_CHARSET_FAMILY==U_EBCDIC_FAMILY
#   define U_UPPER_ORDINAL(x) (((x) < 'J') ? ((x)-'A') : \
                              (((x) < 'S') ? ((x)-'J'+9) : \
                               ((x)-'S'+18)))
#else
#   error Unknown charset family!
#endif

U_INTERNAL int32_t U_EXPORT2
uprv_compareInvEbcdicAsAscii(const char *s1, const char *s2);

#if U_CHARSET_FAMILY==U_ASCII_FAMILY
#   define uprv_compareInvCharsAsAscii(s1, s2) uprv_strcmp(s1, s2)
#elif U_CHARSET_FAMILY==U_EBCDIC_FAMILY
#   define uprv_compareInvCharsAsAscii(s1, s2) uprv_compareInvEbcdicAsAscii(s1, s2)
#else
#   error Unknown charset family!
#endif

#endif
