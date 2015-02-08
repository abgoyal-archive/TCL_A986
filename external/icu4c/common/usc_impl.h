
#ifndef USC_IMPL_H
#define USC_IMPL_H
#include "unicode/utypes.h"
#include "unicode/uscript.h"

struct UScriptRun;

typedef struct UScriptRun UScriptRun;

U_CAPI UScriptRun * U_EXPORT2
uscript_openRun(const UChar *src, int32_t length, UErrorCode *pErrorCode);

U_CAPI void U_EXPORT2
uscript_closeRun(UScriptRun *scriptRun);

U_CAPI void U_EXPORT2
uscript_resetRun(UScriptRun *scriptRun);

U_CAPI void U_EXPORT2
uscript_setRunText(UScriptRun *scriptRun, const UChar *src, int32_t length, UErrorCode *pErrorCode);

U_CAPI UBool U_EXPORT2
uscript_nextRun(UScriptRun *scriptRun, int32_t *pRunStart, int32_t *pRunLimit, UScriptCode *pRunScript);

#endif
