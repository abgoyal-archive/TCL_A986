
#ifndef JAMOTEST_H
#define JAMOTEST_H

#include "unicode/utypes.h"

#if !UCONFIG_NO_TRANSLITERATION

#include "unicode/translit.h"
#include "transtst.h"

class JamoTest : public TransliteratorTest {
public:
    JamoTest();
    virtual ~JamoTest();
private:
    void runIndexedTest(int32_t index, UBool exec, const char* &name,
                        char* par=NULL);

    void TestJamo(void);
    
    void TestRealText(void);

    void TestPiecemeal(void);

    //======================================================================
    // Support methods
    //======================================================================

    // Override TransliteratorTest
    virtual void expectAux(const UnicodeString& tag,
                           const UnicodeString& summary, UBool pass,
                           const UnicodeString& expectedResult);

    // Methods to convert Jamo to/from readable short names,
    // e.g. (Gi) <> U+1100
    static const char* JAMO_NAMES_RULES;
    Transliterator *JAMO_NAME;
    Transliterator *NAME_JAMO;
    UnicodeString nameToJamo(const UnicodeString& input);
    UnicodeString jamoToName(const UnicodeString& input);
};

#endif /* #if !UCONFIG_NO_TRANSLITERATION */

#endif
