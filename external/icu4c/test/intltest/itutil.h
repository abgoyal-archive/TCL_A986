

#ifndef _INTLTESTUTILITIES
#define _INTLTESTUTILITIES

#include "intltest.h"

class IntlTestUtilities: public IntlTest {
public:
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

class ErrorCodeTest: public IntlTest {
public:
    void runIndexedTest(int32_t index, UBool exec, const char* &name, char* par = NULL);
    void TestErrorCode();
    void TestSubclass();
};

#endif
