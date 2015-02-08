#ifndef TESTUTIL_H
#define TESTUTIL_H

#include "intltest.h"

class TestUtility {

public:
    static UnicodeString &appendHex(UnicodeString &buf, UChar32 ch);

    static UnicodeString hex(UChar32 ch);

    static UnicodeString hex(const UnicodeString& s);

    static UnicodeString hex(const UnicodeString& s, UChar sep);

	static UnicodeString hex(const uint8_t* bytes, int32_t len);

private:

    TestUtility() {} // Prevent instantiation
};

#endif
