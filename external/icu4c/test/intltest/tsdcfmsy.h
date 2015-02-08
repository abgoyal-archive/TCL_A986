
#ifndef _INTLTESTDECIMALFORMATSYMBOLS
#define _INTLTESTDECIMALFORMATSYMBOLS

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "unicode/unistr.h"
#include "unicode/dcfmtsym.h"
#include "intltest.h"
class IntlTestDecimalFormatSymbols: public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );  

private:
    /**
     * Test the API of DecimalFormatSymbols; primarily a simple get/set set.
     */
    void testSymbols(/*char *par*/);

     /** helper functions**/
    void Verify(double value, const UnicodeString& pattern, DecimalFormatSymbols sym, const UnicodeString& expected);
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
