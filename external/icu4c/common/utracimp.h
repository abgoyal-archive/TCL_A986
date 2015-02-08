
#ifndef __UTRACIMP_H__
#define __UTRACIMP_H__

#include "unicode/utrace.h"
#include <stdarg.h>

U_CDECL_BEGIN

#ifdef UTRACE_IMPL
U_EXPORT int32_t
#else
U_CFUNC U_COMMON_API int32_t
#endif
utrace_level;


typedef enum UTraceExitVal {
    /** The traced function returns no value  @internal */
    UTRACE_EXITV_NONE   = 0,
    /** The traced function returns an int32_t, or compatible, type.  @internal */
    UTRACE_EXITV_I32    = 1,
    /** The traced function returns a pointer  @internal */
    UTRACE_EXITV_PTR    = 2,
    /** The traced function returns a UBool  @internal */
    UTRACE_EXITV_BOOL   = 3,
    /** Mask to extract the return type values from a UTraceExitVal  @internal */
    UTRACE_EXITV_MASK   = 0xf,
    /** Bit indicating that the traced function includes a UErrorCode parameter  @internal */
    UTRACE_EXITV_STATUS = 0x10
} UTraceExitVal;

U_CAPI void U_EXPORT2
utrace_entry(int32_t fnNumber);

U_CAPI void U_EXPORT2
utrace_exit(int32_t fnNumber, int32_t returnType, ...);


U_CAPI void U_EXPORT2
utrace_data(int32_t utraceFnNumber, int32_t level, const char *fmt, ...);

U_CDECL_END

#if U_ENABLE_TRACING

#define UTRACE_LEVEL(level) (utrace_getLevel()>=(level))

#define UTRACE_TRACED_ENTRY 0x80000000

#define UTRACE_ENTRY(fnNumber) \
    int32_t utraceFnNumber=(fnNumber); \
    if(utrace_getLevel()>=UTRACE_INFO) { \
        utrace_entry(fnNumber); \
        utraceFnNumber |= UTRACE_TRACED_ENTRY; \
    }


#define UTRACE_ENTRY_OC(fnNumber) \
    int32_t utraceFnNumber=(fnNumber); \
    if(utrace_getLevel()>=UTRACE_OPEN_CLOSE) { \
        utrace_entry(fnNumber); \
        utraceFnNumber |= UTRACE_TRACED_ENTRY; \
    }

#define UTRACE_EXIT() \
    {if(utraceFnNumber & UTRACE_TRACED_ENTRY) { \
        utrace_exit(utraceFnNumber & ~UTRACE_TRACED_ENTRY, UTRACE_EXITV_NONE); \
    }}

#define UTRACE_EXIT_VALUE(val) \
    {if(utraceFnNumber & UTRACE_TRACED_ENTRY) { \
        utrace_exit(utraceFnNumber & ~UTRACE_TRACED_ENTRY, UTRACE_EXITV_I32, val); \
    }}

#define UTRACE_EXIT_STATUS(status) \
    {if(utraceFnNumber & UTRACE_TRACED_ENTRY) { \
        utrace_exit(utraceFnNumber & ~UTRACE_TRACED_ENTRY, UTRACE_EXITV_STATUS, status); \
    }}

#define UTRACE_EXIT_VALUE_STATUS(val, status) \
    {if(utraceFnNumber & UTRACE_TRACED_ENTRY) { \
        utrace_exit(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (UTRACE_EXITV_I32 | UTRACE_EXITV_STATUS), val, status); \
    }}

#define UTRACE_EXIT_PTR_STATUS(ptr, status) \
    {if(utraceFnNumber & UTRACE_TRACED_ENTRY) { \
        utrace_exit(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (UTRACE_EXITV_PTR | UTRACE_EXITV_STATUS), ptr, status); \
    }}

#define UTRACE_DATA0(level, fmt) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt)); \
    }

#define UTRACE_DATA1(level, fmt, a) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY , (level), (fmt), (a)); \
    }

#define UTRACE_DATA2(level, fmt, a, b) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY , (level), (fmt), (a), (b)); \
    }

#define UTRACE_DATA3(level, fmt, a, b, c) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c)); \
    }

#define UTRACE_DATA4(level, fmt, a, b, c, d) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d)); \
    }

#define UTRACE_DATA5(level, fmt, a, b, c, d, e) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d), (e)); \
    }

#define UTRACE_DATA6(level, fmt, a, b, c, d, e, f) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d), (e), (f)); \
    }

#define UTRACE_DATA7(level, fmt, a, b, c, d, e, f, g) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d), (e), (f), (g)); \
    }

#define UTRACE_DATA8(level, fmt, a, b, c, d, e, f, g, h) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d), (e), (f), (g), (h)); \
    }

#define UTRACE_DATA9(level, fmt, a, b, c, d, e, f, g, h, i) \
    if(UTRACE_LEVEL(level)) { \
        utrace_data(utraceFnNumber & ~UTRACE_TRACED_ENTRY, (level), (fmt), (a), (b), (c), (d), (e), (f), (g), (h), (i)); \
    }

#else


#define UTRACE_LEVEL(level) 0
#define UTRACE_ENTRY(fnNumber)
#define UTRACE_ENTRY_OC(fnNumber)
#define UTRACE_EXIT()
#define UTRACE_EXIT_VALUE(val)
#define UTRACE_EXIT_STATUS(status)
#define UTRACE_EXIT_VALUE_STATUS(val, status)
#define UTRACE_EXIT_PTR_STATUS(ptr, status)
#define UTRACE_DATA0(level, fmt)
#define UTRACE_DATA1(level, fmt, a)
#define UTRACE_DATA2(level, fmt, a, b)
#define UTRACE_DATA3(level, fmt, a, b, c)
#define UTRACE_DATA4(level, fmt, a, b, c, d)
#define UTRACE_DATA5(level, fmt, a, b, c, d, e)
#define UTRACE_DATA6(level, fmt, a, b, c, d, e, f)
#define UTRACE_DATA7(level, fmt, a, b, c, d, e, f, g)
#define UTRACE_DATA8(level, fmt, a, b, c, d, e, f, g, h)
#define UTRACE_DATA9(level, fmt, a, b, c, d, e, f, g, h, i)

#endif

#endif
