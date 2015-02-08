
#ifndef _UCURR_IMP_H_
#define _UCURR_IMP_H_

#include "unicode/utypes.h"
#include "unicode/unistr.h"
#include "unicode/parsepos.h"

U_CFUNC void
uprv_getStaticCurrencyName(const UChar* iso, const char* loc,
                           U_NAMESPACE_QUALIFIER UnicodeString& result, UErrorCode& ec);

U_CFUNC void
uprv_parseCurrency(const char* locale,
                   const U_NAMESPACE_QUALIFIER UnicodeString& text,
                   U_NAMESPACE_QUALIFIER ParsePosition& pos,
                   int8_t type,
                   UChar* result,
                   UErrorCode& ec);

#endif /* #ifndef _UCURR_IMP_H_ */

//eof
