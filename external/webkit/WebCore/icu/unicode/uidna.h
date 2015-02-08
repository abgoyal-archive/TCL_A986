

#ifndef __UIDNA_H__
#define __UIDNA_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_IDNA

#include "unicode/parseerr.h"
  

#ifndef U_HIDE_DRAFT_API

#define UIDNA_DEFAULT          0x0000
#define UIDNA_ALLOW_UNASSIGNED 0x0001
#define UIDNA_USE_STD3_RULES   0x0002

#endif /*U_HIDE_DRAFT_API*/
    
U_STABLE int32_t U_EXPORT2
uidna_toASCII(const UChar* src, int32_t srcLength, 
              UChar* dest, int32_t destCapacity,
              int32_t options,
              UParseError* parseError,
              UErrorCode* status);


U_STABLE int32_t U_EXPORT2
uidna_toUnicode(const UChar* src, int32_t srcLength,
                UChar* dest, int32_t destCapacity,
                int32_t options,
                UParseError* parseError,
                UErrorCode* status);


U_STABLE int32_t U_EXPORT2
uidna_IDNToASCII(  const UChar* src, int32_t srcLength,
                   UChar* dest, int32_t destCapacity,
                   int32_t options,
                   UParseError* parseError,
                   UErrorCode* status);

U_STABLE int32_t U_EXPORT2
uidna_IDNToUnicode(  const UChar* src, int32_t srcLength,
                     UChar* dest, int32_t destCapacity,
                     int32_t options,
                     UParseError* parseError,
                     UErrorCode* status);

U_STABLE int32_t U_EXPORT2
uidna_compare(  const UChar *s1, int32_t length1,
                const UChar *s2, int32_t length2,
                int32_t options,
                UErrorCode* status);

#endif /* #if !UCONFIG_NO_IDNA */

#endif
