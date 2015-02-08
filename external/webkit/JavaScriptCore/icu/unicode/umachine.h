

#ifndef __UMACHINE_H__
#define __UMACHINE_H__


/*==========================================================================*/
/* Include platform-dependent definitions                                   */
/* which are contained in the platform-specific file platform.h             */
/*==========================================================================*/

#if defined(WIN32) || defined(_WIN32) || defined(WIN64) || defined(_WIN64)
#   include "unicode/pwin32.h"
#else
#   include "unicode/platform.h"
#endif

#include <stddef.h>

/*==========================================================================*/
/* XP_CPLUSPLUS is a cross-platform symbol which should be defined when     */
/* using C++.  It should not be defined when compiling under C.             */
/*==========================================================================*/

#ifdef __cplusplus
#   ifndef XP_CPLUSPLUS
#       define XP_CPLUSPLUS
#   endif
#else
#   undef XP_CPLUSPLUS
#endif

/*==========================================================================*/
/* For C wrappers, we use the symbol U_STABLE.                                */
/* This works properly if the includer is C or C++.                         */
/* Functions are declared   U_STABLE return-type U_EXPORT2 function-name()... */
/*==========================================================================*/




#ifdef XP_CPLUSPLUS
#   define U_CFUNC extern "C"
#   define U_CDECL_BEGIN extern "C" {
#   define U_CDECL_END   }
#else
#   define U_CFUNC extern
#   define U_CDECL_BEGIN
#   define U_CDECL_END
#endif





/* Define namespace symbols if the compiler supports it. */
#if U_HAVE_NAMESPACE
#   define U_NAMESPACE_BEGIN namespace U_ICU_NAMESPACE {
#   define U_NAMESPACE_END  }
#   define U_NAMESPACE_USE using namespace U_ICU_NAMESPACE;
#   define U_NAMESPACE_QUALIFIER U_ICU_NAMESPACE::
#else
#   define U_NAMESPACE_BEGIN
#   define U_NAMESPACE_END
#   define U_NAMESPACE_USE
#   define U_NAMESPACE_QUALIFIER
#endif

/** This is used to declare a function as a public ICU C API @stable ICU 2.0*/
#define U_CAPI U_CFUNC U_EXPORT
#define U_STABLE U_CAPI
#define U_DRAFT  U_CAPI
#define U_DEPRECATED U_CAPI
#define U_OBSOLETE U_CAPI
#define U_INTERNAL U_CAPI

/*==========================================================================*/
/* limits for int32_t etc., like in POSIX inttypes.h                        */
/*==========================================================================*/

#ifndef INT8_MIN
/** The smallest value an 8 bit signed integer can hold @stable ICU 2.0 */
#   define INT8_MIN        ((int8_t)(-128))
#endif
#ifndef INT16_MIN
/** The smallest value a 16 bit signed integer can hold @stable ICU 2.0 */
#   define INT16_MIN       ((int16_t)(-32767-1))
#endif
#ifndef INT32_MIN
/** The smallest value a 32 bit signed integer can hold @stable ICU 2.0 */
#   define INT32_MIN       ((int32_t)(-2147483647-1))
#endif

#ifndef INT8_MAX
/** The largest value an 8 bit signed integer can hold @stable ICU 2.0 */
#   define INT8_MAX        ((int8_t)(127))
#endif
#ifndef INT16_MAX
/** The largest value a 16 bit signed integer can hold @stable ICU 2.0 */
#   define INT16_MAX       ((int16_t)(32767))
#endif
#ifndef INT32_MAX
/** The largest value a 32 bit signed integer can hold @stable ICU 2.0 */
#   define INT32_MAX       ((int32_t)(2147483647))
#endif

#ifndef UINT8_MAX
/** The largest value an 8 bit unsigned integer can hold @stable ICU 2.0 */
#   define UINT8_MAX       ((uint8_t)(255U))
#endif
#ifndef UINT16_MAX
/** The largest value a 16 bit unsigned integer can hold @stable ICU 2.0 */
#   define UINT16_MAX      ((uint16_t)(65535U))
#endif
#ifndef UINT32_MAX
/** The largest value a 32 bit unsigned integer can hold @stable ICU 2.0 */
#   define UINT32_MAX      ((uint32_t)(4294967295U))
#endif

#if defined(U_INT64_T_UNAVAILABLE)
# error int64_t is required for decimal format and rule-based number format.
#else
# ifndef INT64_C
#   define INT64_C(c) c ## LL
# endif
# ifndef UINT64_C
#   define UINT64_C(c) c ## ULL
# endif
# ifndef U_INT64_MIN
/** The smallest value a 64 bit signed integer can hold @stable ICU 2.8 */
#     define U_INT64_MIN       ((int64_t)(INT64_C(-9223372036854775807)-1))
# endif
# ifndef U_INT64_MAX
/** The largest value a 64 bit signed integer can hold @stable ICU 2.8 */
#     define U_INT64_MAX       ((int64_t)(INT64_C(9223372036854775807)))
# endif
# ifndef U_UINT64_MAX
/** The largest value a 64 bit unsigned integer can hold @stable ICU 2.8 */
#     define U_UINT64_MAX      ((uint64_t)(UINT64_C(18446744073709551615)))
# endif
#endif

/*==========================================================================*/
/* Boolean data type                                                        */
/*==========================================================================*/

/** The ICU boolean type @stable ICU 2.0 */
typedef int8_t UBool;

#ifndef TRUE
/** The TRUE value of a UBool @stable ICU 2.0 */
#   define TRUE  1
#endif
#ifndef FALSE
/** The FALSE value of a UBool @stable ICU 2.0 */
#   define FALSE 0
#endif


/*==========================================================================*/
/* Unicode data types                                                       */
/*==========================================================================*/

/* wchar_t-related definitions -------------------------------------------- */

#ifndef U_HAVE_WCHAR_H
#   define U_HAVE_WCHAR_H 1
#endif

#if U_SIZEOF_WCHAR_T==0
#   undef U_SIZEOF_WCHAR_T
#   define U_SIZEOF_WCHAR_T 4
#endif

#if !defined(U_WCHAR_IS_UTF16) && !defined(U_WCHAR_IS_UTF32)
#   ifdef __STDC_ISO_10646__ 
#       if (U_SIZEOF_WCHAR_T==2)
#           define U_WCHAR_IS_UTF16
#       elif (U_SIZEOF_WCHAR_T==4)
#           define  U_WCHAR_IS_UTF32
#       endif
#   elif defined __UCS2__
#       if (__OS390__ || __OS400__) && (U_SIZEOF_WCHAR_T==2)
#           define U_WCHAR_IS_UTF16
#       endif
#   elif defined __UCS4__
#       if (U_SIZEOF_WCHAR_T==4)
#           define U_WCHAR_IS_UTF32
#       endif
#   elif defined(WIN32) || defined(_WIN32) || defined(WIN64) || defined(_WIN64)
#       define U_WCHAR_IS_UTF16    
#   endif
#endif

/* UChar and UChar32 definitions -------------------------------------------- */

/** Number of bytes in a UChar. @stable ICU 2.0 */
#define U_SIZEOF_UCHAR 2


/* Define UChar to be compatible with wchar_t if possible. */
#if U_SIZEOF_WCHAR_T==2
    typedef wchar_t UChar;
#else
    typedef uint16_t UChar;
#endif

typedef int32_t UChar32;

/*==========================================================================*/
/* U_INLINE and U_ALIGN_CODE   Set default values if these are not already  */
/*                             defined.  Definitions normally are in        */
/*                             platform.h or the corresponding file for     */
/*                             the OS in use.                               */
/*==========================================================================*/

#ifndef U_ALIGN_CODE
#   define U_ALIGN_CODE(n)
#endif

#ifndef U_INLINE
#   define U_INLINE
#endif

#include "unicode/urename.h"

#endif
