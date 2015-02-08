
#ifndef INTLTESTRBBI_H
#define INTLTESTRBBI_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_BREAK_ITERATION

#include "intltest.h"


class IntlTestRBBI: public IntlTest {
public:
    void runIndexedTest( int32_t index, UBool exec, const char* &name, char* par = NULL );
};

#endif /* #if !UCONFIG_NO_BREAK_ITERATION */

#endif
