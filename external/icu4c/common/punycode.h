
/* This ICU code derived from: */

#ifndef __PUNYCODE_H__
#define __PUNYCODE_H__

#include "unicode/utypes.h"

#if !UCONFIG_NO_IDNA

U_CFUNC int32_t
u_strToPunycode(const UChar *src, int32_t srcLength,
                UChar *dest, int32_t destCapacity,
                const UBool *caseFlags,
                UErrorCode *pErrorCode);

U_CFUNC int32_t
u_strFromPunycode(const UChar *src, int32_t srcLength,
                  UChar *dest, int32_t destCapacity,
                  UBool *caseFlags,
                  UErrorCode *pErrorCode);

#endif /* #if !UCONFIG_NO_IDNA */

#endif

