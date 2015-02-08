

#ifndef _INTLTESTMAJOR
#define _INTLTESTMAJOR


#include "intltest.h"


class MajorTestLevel: public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

class IntlTestNormalize: public IntlTest {
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

#endif
