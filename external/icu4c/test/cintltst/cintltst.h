
#ifndef _CINTLTST
#define _CINTLTST

#include "unicode/utypes.h"
#include "unicode/putil.h"
#include "unicode/ctest.h"
#include <stdlib.h>

#ifndef U_USE_DEPRECATED_API
#define U_USE_DEPRECATED_API 1
#endif

U_CFUNC void addAllTests(TestNode** root);

U_CFUNC const char* ctest_dataOutDir(void);

U_CFUNC const char* ctest_dataSrcDir(void);

U_CFUNC UChar* CharsToUChars(const char* chars);

 
U_CFUNC char *austrdup(const UChar* unichars);
U_CFUNC char *aescstrdup(const UChar* unichars, int32_t length);
U_CFUNC void *ctst_malloc(size_t size);
U_CFUNC void ctst_freeAll(void);

U_CFUNC const char* loadTestData(UErrorCode* err);

#define myErrorName(errorCode) u_errorName(errorCode)


U_CFUNC void ctest_setTimeZone(const char *optionalTimeZone, UErrorCode *status);
U_CFUNC void ctest_resetTimeZone(void);

U_CFUNC UBool ctest_resetICU(void);

U_CFUNC UBool assertSuccess(const char* msg, UErrorCode* ec);

U_CFUNC UBool assertTrue(const char* msg, int condition);

U_CFUNC UBool assertEquals(const char* msg, const char* expectedString,
                           const char* actualString);

U_CFUNC UBool isICUVersionAtLeast(const UVersionInfo x);

#endif
