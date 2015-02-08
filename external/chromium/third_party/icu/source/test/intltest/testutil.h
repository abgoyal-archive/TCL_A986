#ifndef TESTUTIL_H
#define TESTUTIL_H

#include "intltest.h"

class TestUtility {

public:

    static UnicodeString hex(UChar ch);

    static UnicodeString hex(const UnicodeString& s);

    static UnicodeString hex(const UnicodeString& s, UChar sep);

private:

    TestUtility() {} // Prevent instantiation
};

#endif
