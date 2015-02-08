

#ifndef REPTEST_H
#define REPTEST_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"
#include "intltest.h"

class ReplaceableTest : public IntlTest {
public:
    void runIndexedTest(int32_t index, UBool exec, const char* &name, char* par=NULL);

    /*Tests the Replaceable class according to the API documentation. */
    void TestReplaceableClass(void);
private:
    void check( const UnicodeString& transliteratorName, 
                const UnicodeString& test, 
                const UnicodeString& shouldProduceStyles);
};

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
