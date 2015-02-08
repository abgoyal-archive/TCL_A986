
#ifndef __UTRACE_H__
#define __UTRACE_H__

#include <stdarg.h>
#include "unicode/utypes.h"

 
U_CDECL_BEGIN

typedef enum UTraceLevel {
    /** Disable all tracing  @stable ICU 2.8*/
    UTRACE_OFF=-1,
    /** Trace error conditions only  @stable ICU 2.8*/
    UTRACE_ERROR=0,
    /** Trace errors and warnings  @stable ICU 2.8*/
    UTRACE_WARNING=3,
    /** Trace opens and closes of ICU services  @stable ICU 2.8*/
    UTRACE_OPEN_CLOSE=5,
    /** Trace an intermediate number of ICU operations  @stable ICU 2.8*/
    UTRACE_INFO=7,
    /** Trace the maximum number of ICU operations  @stable ICU 2.8*/
    UTRACE_VERBOSE=9
} UTraceLevel;

typedef enum UTraceFunctionNumber {
    UTRACE_FUNCTION_START=0,
    UTRACE_U_INIT=UTRACE_FUNCTION_START,
    UTRACE_U_CLEANUP,
    UTRACE_FUNCTION_LIMIT,

    UTRACE_CONVERSION_START=0x1000,
    UTRACE_UCNV_OPEN=UTRACE_CONVERSION_START,
    UTRACE_UCNV_OPEN_PACKAGE,
    UTRACE_UCNV_OPEN_ALGORITHMIC,
    UTRACE_UCNV_CLONE,
    UTRACE_UCNV_CLOSE,
    UTRACE_UCNV_FLUSH_CACHE,
    UTRACE_UCNV_LOAD,
    UTRACE_UCNV_UNLOAD,
    UTRACE_CONVERSION_LIMIT,

    UTRACE_COLLATION_START=0x2000,
    UTRACE_UCOL_OPEN=UTRACE_COLLATION_START,
    UTRACE_UCOL_CLOSE,
    UTRACE_UCOL_STRCOLL,
    UTRACE_UCOL_GET_SORTKEY,
    UTRACE_UCOL_GETLOCALE,
    UTRACE_UCOL_NEXTSORTKEYPART,
    UTRACE_UCOL_STRCOLLITER,
    UTRACE_UCOL_OPEN_FROM_SHORT_STRING,
    UTRACE_COLLATION_LIMIT
} UTraceFunctionNumber;

U_STABLE void U_EXPORT2
utrace_setLevel(int32_t traceLevel);

U_STABLE int32_t U_EXPORT2
utrace_getLevel(void);

/* Trace function pointers types  ----------------------------- */

typedef void U_CALLCONV
UTraceEntry(const void *context, int32_t fnNumber);

typedef void U_CALLCONV
UTraceExit(const void *context, int32_t fnNumber, 
           const char *fmt, va_list args);

typedef void U_CALLCONV
UTraceData(const void *context, int32_t fnNumber, int32_t level,
           const char *fmt, va_list args);

U_STABLE void U_EXPORT2
utrace_setFunctions(const void *context,
                    UTraceEntry *e, UTraceExit *x, UTraceData *d);

U_STABLE void U_EXPORT2
utrace_getFunctions(const void **context,
                    UTraceEntry **e, UTraceExit **x, UTraceData **d);






U_STABLE int32_t U_EXPORT2
utrace_vformat(char *outBuf, int32_t capacity,
              int32_t indent, const char *fmt,  va_list args);

U_STABLE int32_t U_EXPORT2
utrace_format(char *outBuf, int32_t capacity,
              int32_t indent, const char *fmt,  ...);



/* Trace function numbers --------------------------------------------------- */

U_STABLE const char * U_EXPORT2
utrace_functionName(int32_t fnNumber);

U_CDECL_END

#endif
