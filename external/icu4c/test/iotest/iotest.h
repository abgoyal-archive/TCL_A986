
#ifndef IOTEST_H
#define IOTEST_H 1

#include "unicode/utypes.h"
#include "unicode/ctest.h"

U_CFUNC void
addStringTest(TestNode** root);

U_CFUNC void
addFileTest(TestNode** root);

U_CFUNC void
addTranslitTest(TestNode** root);

U_CFUNC void
addStreamTests(TestNode** root);

U_CDECL_BEGIN
extern const UChar NEW_LINE[];
extern const char C_NEW_LINE[];
extern const char *STANDARD_TEST_FILE;
U_CDECL_END

#define STANDARD_TEST_NUM_RANGE 1000


#endif
