
#ifndef CTEST_H
#define CTEST_H

#include "unicode/testtype.h"
#include "unicode/utrace.h"


/* prototypes *********************************/

U_CDECL_BEGIN
typedef void (U_CALLCONV *TestFunctionPtr)(void);
typedef int (U_CALLCONV *ArgHandlerPtr)(int arg, int argc, const char* const argv[], void *context);
typedef struct TestNode TestNode;
U_CDECL_END

extern T_CTEST_EXPORT_API int REPEAT_TESTS;

extern T_CTEST_EXPORT_API int VERBOSITY;

extern T_CTEST_EXPORT_API int ERR_MSG;

extern T_CTEST_EXPORT_API int QUICK;

extern T_CTEST_EXPORT_API int WARN_ON_MISSING_DATA;

extern T_CTEST_EXPORT_API UTraceLevel ICU_TRACE;

extern T_CTEST_EXPORT_API size_t MAX_MEMORY_ALLOCATION;

extern T_CTEST_EXPORT_API int32_t ALLOCATION_COUNT;


T_CTEST_API void T_CTEST_EXPORT2
showTests ( const TestNode *root);

T_CTEST_API void T_CTEST_EXPORT2
runTests ( const TestNode* root);

T_CTEST_API void T_CTEST_EXPORT2
addTest(TestNode** root,
        TestFunctionPtr test,
        const char *path);

T_CTEST_API void T_CTEST_EXPORT2
cleanUpTestTree(TestNode *tn);

T_CTEST_API const TestNode* T_CTEST_EXPORT2
getTest(const TestNode* root,
        const char *path);


T_CTEST_API void T_CTEST_EXPORT2
log_err(const char* pattern, ...);

T_CTEST_API void T_CTEST_EXPORT2
log_err_status(UErrorCode status, const char* pattern, ...);
T_CTEST_API void T_CTEST_EXPORT2
log_info(const char* pattern, ...);

T_CTEST_API void T_CTEST_EXPORT2
vlog_info(const char *prefix, const char *pattern, va_list ap);

T_CTEST_API void T_CTEST_EXPORT2
log_verbose(const char* pattern, ...);

T_CTEST_API void T_CTEST_EXPORT2
log_data_err(const char *pattern, ...);

T_CTEST_API int T_CTEST_EXPORT2 
initArgs( int argc, const char* const argv[], ArgHandlerPtr argHandler, void *context);

T_CTEST_API int T_CTEST_EXPORT2 
runTestRequest(const TestNode* root,
            int argc,
            const char* const argv[]);


T_CTEST_API const char* T_CTEST_EXPORT2
getTestName(void);



#endif
