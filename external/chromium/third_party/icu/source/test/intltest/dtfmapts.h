
#ifndef _INTLTESTDATEFORMATAPI
#define _INTLTESTDATEFORMATAPI

#include "unicode/utypes.h"

#if !UCONFIG_NO_FORMATTING

#include "intltest.h"


class IntlTestDateFormatAPI: public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );  

private:
    /**
     * Tests basic functionality of various generic API methods in DateFormat 
     */
    void testAPI(/* char* par */);
    /**
     * Test that the equals method works correctly.
     */
    void TestEquals(void);

    /**
     * Test that no parse or format methods are hidden.
     */
    void TestNameHiding(void);
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif
