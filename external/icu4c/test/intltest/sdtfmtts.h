
#ifndef _INTLTESTSIMPLEDATEFORMATAPI
#define _INTLTESTSIMPLEDATEFORMATAPI

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "intltest.h"

class IntlTestSimpleDateFormatAPI : public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );  

private:
    /**
     * Performs tests on many API functions, see detailed comments in source code
     **/
    void testAPI(/* char* par */);
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
