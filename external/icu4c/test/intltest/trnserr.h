

#ifndef TRNSERR_H
#define TRNSERR_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"
#include "intltest.h"

class TransliteratorErrorTest : public IntlTest {
public:
    void runIndexedTest(int32_t index, UBool exec, const char* &name, char* par=NULL);

    /*Tests the returned error codes on all the APIs according to the API documentation. */
    void TestTransliteratorErrors(void);
    
    void TestUnicodeSetErrors(void);

    //void TestUniToHexErrors(void);

    void TestRBTErrors(void);

    //void TestHexToUniErrors(void);

    // JitterBug 4452, for coverage.  The reason to put this method here is 
    //  this class is comparable smaller than other Transliterator*Test classes
    void TestCoverage(void);

};

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
