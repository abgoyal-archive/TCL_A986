
#include "intltest.h"
#include "unicode/locdspnm.h"

class LocaleDisplayNamesTest: public IntlTest {
public:
    LocaleDisplayNamesTest();
    virtual ~LocaleDisplayNamesTest();
    
    void runIndexedTest(int32_t index, UBool exec, const char* &name, char* par = NULL);

#if !UCONFIG_NO_FORMATTING
    /**
     * Test methods to set and get data fields
     **/
    void TestCreate(void);
    void TestCreateDialect(void);
    void TestWithKeywordsAndEverything(void);
    void TestUldnOpen(void);
    void TestUldnOpenDialect(void);
    void TestUldnWithKeywordsAndEverything(void);
    void TestUldnComponents(void);
    void TestRootEtc(void);
#endif
};
