

#ifndef __STRTEST_H__
#define __STRTEST_H__

#include "intltest.h"

class StringTest : public IntlTest {
public:
    StringTest() {}
    virtual ~StringTest();

    void runIndexedTest(int32_t index, UBool exec, const char *&name, char *par=NULL);

private:
    void TestEndian(void);
    void TestSizeofTypes(void);
    void TestCharsetFamily(void);
    void TestStdNamespaceQualifier();
    void TestUsingStdNamespace();
    void TestStringPiece();
    void TestByteSink();
    void TestCheckedArrayByteSink();
    void TestStringByteSink();
    void TestSTLCompatibility();
};

#endif
