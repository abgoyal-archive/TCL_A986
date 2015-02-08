
#ifndef _INTLTESTNUMBERFORMATAPI
#define _INTLTESTNUMBERFORMATAPI

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "intltest.h"


class IntlTestNumberFormatAPI: public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );  

private:
    /**
     * executes tests of API functions, see detailed comments in source code
     **/
    void testAPI(/* char* par */);
    void testRegistration();
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
