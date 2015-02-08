
#ifndef __IDNAREF_H__
#define __IDNAREF_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_IDNA

#include "unicode/parseerr.h"

#define IDNAREF_DEFAULT          0x0000
#define IDNAREF_ALLOW_UNASSIGNED 0x0001
#define IDNAREF_USE_STD3_RULES   0x0002
    
U_CFUNC int32_t U_EXPORT2
idnaref_toASCII(const UChar* src, int32_t srcLength, 
              UChar* dest, int32_t destCapacity,
              int32_t options,
              UParseError* parseError,
              UErrorCode* status);


U_CFUNC int32_t U_EXPORT2
idnaref_toUnicode(const UChar* src, int32_t srcLength,
                UChar* dest, int32_t destCapacity,
                int32_t options,
                UParseError* parseError,
                UErrorCode* status);


U_CFUNC int32_t U_EXPORT2
idnaref_IDNToASCII(  const UChar* src, int32_t srcLength,
                   UChar* dest, int32_t destCapacity,
                   int32_t options,
                   UParseError* parseError,
                   UErrorCode* status);

U_CFUNC int32_t U_EXPORT2
idnaref_IDNToUnicode(  const UChar* src, int32_t srcLength,
                     UChar* dest, int32_t destCapacity,
                     int32_t options,
                     UParseError* parseError,
                     UErrorCode* status);

U_CFUNC int32_t U_EXPORT2
idnaref_compare(  const UChar *s1, int32_t length1,
                const UChar *s2, int32_t length2,
                int32_t options,
                UErrorCode* status);

#endif /* #if !UCONFIG_NO_IDNA */

#endif
