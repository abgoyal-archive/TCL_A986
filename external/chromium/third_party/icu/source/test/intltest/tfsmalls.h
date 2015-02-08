

#ifndef TESTFORMATSMALLCLASSES_H
#define TESTFORMATSMALLCLASSES_H

#include "intltest.h"

class TestFormatSmallClasses: public IntlTest {
    /**
     * runs tests in 4 local routines,
     * performs test for API and funtionalty of 3 smaller format classes:
     *    ParsePosition in test_ParsePosition(),
     *    FieldPosition in test_FieldPosition(),
     *    Formattable in test_Formattable().
     **/    
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

#endif
