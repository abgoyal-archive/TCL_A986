/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 2 -*- */


#ifndef prtypes_h___
#define prtypes_h___

#ifdef MDCPUCFG
#include MDCPUCFG
#else
#include "base/third_party/nspr/prcpucfg.h"
#endif

#include <stddef.h>

#if 1


#define PR_EXPORT(__type) extern __type
#define PR_EXPORT_DATA(__type) extern __type
#define PR_IMPORT(__type) extern __type
#define PR_IMPORT_DATA(__type) extern __type

#define PR_EXTERN(__type) extern __type
#define PR_IMPLEMENT(__type) __type
#define PR_EXTERN_DATA(__type) extern __type
#define PR_IMPLEMENT_DATA(__type) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(WIN32)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) __declspec(dllimport) __type
#define PR_IMPORT_DATA(__type) __declspec(dllimport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_BEOS)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) extern __declspec(dllexport) __type
#define PR_IMPORT_DATA(__type) extern __declspec(dllexport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(WIN16)

#define PR_CALLBACK_DECL        __cdecl

#if defined(_WINDLL)
#define PR_EXPORT(__type) extern __type _cdecl _export _loadds
#define PR_IMPORT(__type) extern __type _cdecl _export _loadds
#define PR_EXPORT_DATA(__type) extern __type _export
#define PR_IMPORT_DATA(__type) extern __type _export

#define PR_EXTERN(__type) extern __type _cdecl _export _loadds
#define PR_IMPLEMENT(__type) __type _cdecl _export _loadds
#define PR_EXTERN_DATA(__type) extern __type _export
#define PR_IMPLEMENT_DATA(__type) __type _export

#define PR_CALLBACK             __cdecl __loadds
#define PR_STATIC_CALLBACK(__x) static __x PR_CALLBACK

#else /* this must be .EXE */
#define PR_EXPORT(__type) extern __type _cdecl _export
#define PR_IMPORT(__type) extern __type _cdecl _export
#define PR_EXPORT_DATA(__type) extern __type _export
#define PR_IMPORT_DATA(__type) extern __type _export

#define PR_EXTERN(__type) extern __type _cdecl _export
#define PR_IMPLEMENT(__type) __type _cdecl _export
#define PR_EXTERN_DATA(__type) extern __type _export
#define PR_IMPLEMENT_DATA(__type) __type _export

#define PR_CALLBACK             __cdecl __loadds
#define PR_STATIC_CALLBACK(__x) __x PR_CALLBACK
#endif /* _WINDLL */

#elif defined(XP_MAC)

#define PR_EXPORT(__type) extern __declspec(export) __type
#define PR_EXPORT_DATA(__type) extern __declspec(export) __type
#define PR_IMPORT(__type) extern __declspec(export) __type
#define PR_IMPORT_DATA(__type) extern __declspec(export) __type

#define PR_EXTERN(__type) extern __declspec(export) __type
#define PR_IMPLEMENT(__type) __declspec(export) __type
#define PR_EXTERN_DATA(__type) extern __declspec(export) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(export) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_OS2) && defined(__declspec)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) extern  __declspec(dllimport) __type
#define PR_IMPORT_DATA(__type) extern __declspec(dllimport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_OS2_VACPP)

#define PR_EXPORT(__type) extern __type
#define PR_EXPORT_DATA(__type) extern __type
#define PR_IMPORT(__type) extern __type
#define PR_IMPORT_DATA(__type) extern __type

#define PR_EXTERN(__type) extern __type
#define PR_IMPLEMENT(__type) __type
#define PR_EXTERN_DATA(__type) extern __type
#define PR_IMPLEMENT_DATA(__type) __type
#define PR_CALLBACK _Optlink
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x PR_CALLBACK

#else /* Unix */

/* GCC 3.3 and later support the visibility attribute. */
#if (__GNUC__ >= 4) || \
    (__GNUC__ == 3 && __GNUC_MINOR__ >= 3)
#define PR_VISIBILITY_DEFAULT __attribute__((visibility("default")))
#else
#define PR_VISIBILITY_DEFAULT
#endif

#define PR_EXPORT(__type) extern PR_VISIBILITY_DEFAULT __type
#define PR_EXPORT_DATA(__type) extern PR_VISIBILITY_DEFAULT __type
#define PR_IMPORT(__type) extern PR_VISIBILITY_DEFAULT __type
#define PR_IMPORT_DATA(__type) extern PR_VISIBILITY_DEFAULT __type

#define PR_EXTERN(__type) extern PR_VISIBILITY_DEFAULT __type
#define PR_IMPLEMENT(__type) PR_VISIBILITY_DEFAULT __type
#define PR_EXTERN_DATA(__type) extern PR_VISIBILITY_DEFAULT __type
#define PR_IMPLEMENT_DATA(__type) PR_VISIBILITY_DEFAULT __type
#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#endif

#if defined(_NSPR_BUILD_)
#define NSPR_API(__type) PR_EXPORT(__type)
#define NSPR_DATA_API(__type) PR_EXPORT_DATA(__type)
#else
#define NSPR_API(__type) PR_IMPORT(__type)
#define NSPR_DATA_API(__type) PR_IMPORT_DATA(__type)
#endif

#define PR_BEGIN_MACRO  do {
#define PR_END_MACRO    } while (0)

#ifdef __cplusplus
#define PR_BEGIN_EXTERN_C       extern "C" {
#define PR_END_EXTERN_C         }
#else
#define PR_BEGIN_EXTERN_C
#define PR_END_EXTERN_C
#endif

#define PR_BIT(n)       ((PRUint32)1 << (n))
#define PR_BITMASK(n)   (PR_BIT(n) - 1)

#define PR_ROUNDUP(x,y) ((((x)+((y)-1))/(y))*(y))
#define PR_MIN(x,y)     ((x)<(y)?(x):(y))
#define PR_MAX(x,y)     ((x)>(y)?(x):(y))
#define PR_ABS(x)       ((x)<0?-(x):(x))

PR_BEGIN_EXTERN_C

#if PR_BYTES_PER_BYTE == 1
typedef unsigned char PRUint8;
#if (defined(HPUX) && defined(__cplusplus) \
        && !defined(__GNUC__) && __cplusplus < 199707L) \
    || (defined(SCO) && defined(__cplusplus) \
        && !defined(__GNUC__) && __cplusplus == 1L)
typedef char PRInt8;
#else
typedef signed char PRInt8;
#endif
#else
#error No suitable type for PRInt8/PRUint8
#endif


#define PR_INT8_MAX 127
#define PR_INT8_MIN (-128)
#define PR_UINT8_MAX 255U

#if PR_BYTES_PER_SHORT == 2
typedef unsigned short PRUint16;
typedef short PRInt16;
#else
#error No suitable type for PRInt16/PRUint16
#endif


#define PR_INT16_MAX 32767
#define PR_INT16_MIN (-32768)
#define PR_UINT16_MAX 65535U

#if PR_BYTES_PER_INT == 4
typedef unsigned int PRUint32;
typedef int PRInt32;
#define PR_INT32(x)  x
#define PR_UINT32(x) x ## U
#elif PR_BYTES_PER_LONG == 4
typedef unsigned long PRUint32;
typedef long PRInt32;
#define PR_INT32(x)  x ## L
#define PR_UINT32(x) x ## UL
#else
#error No suitable type for PRInt32/PRUint32
#endif


#define PR_INT32_MAX PR_INT32(2147483647)
#define PR_INT32_MIN (-PR_INT32_MAX - 1)
#define PR_UINT32_MAX PR_UINT32(4294967295)

#ifdef HAVE_LONG_LONG
#if PR_BYTES_PER_LONG == 8
typedef long PRInt64;
typedef unsigned long PRUint64;
#elif defined(WIN16)
typedef __int64 PRInt64;
typedef unsigned __int64 PRUint64;
#elif defined(WIN32) && !defined(__GNUC__)
typedef __int64  PRInt64;
typedef unsigned __int64 PRUint64;
#else
typedef long long PRInt64;
typedef unsigned long long PRUint64;
#endif /* PR_BYTES_PER_LONG == 8 */
#else  /* !HAVE_LONG_LONG */
typedef struct {
#ifdef IS_LITTLE_ENDIAN
    PRUint32 lo, hi;
#else
    PRUint32 hi, lo;
#endif
} PRInt64;
typedef PRInt64 PRUint64;
#endif /* !HAVE_LONG_LONG */

#if PR_BYTES_PER_INT >= 2
typedef int PRIntn;
typedef unsigned int PRUintn;
#else
#error 'sizeof(int)' not sufficient for platform use
#endif

typedef double          PRFloat64;

typedef size_t PRSize;


typedef PRInt32 PROffset32;
typedef PRInt64 PROffset64;

typedef ptrdiff_t PRPtrdiff;

#ifdef _WIN64
typedef unsigned __int64 PRUptrdiff;
#else
typedef unsigned long PRUptrdiff;
#endif

typedef PRIntn PRBool;
#define PR_TRUE 1
#define PR_FALSE 0

typedef PRUint8 PRPackedBool;

typedef enum { PR_FAILURE = -1, PR_SUCCESS = 0 } PRStatus;

#ifndef __PRUNICHAR__
#define __PRUNICHAR__
#if defined(WIN32) || defined(XP_MAC)
typedef wchar_t PRUnichar;
#else
typedef PRUint16 PRUnichar;
#endif
#endif

#ifdef _WIN64
typedef __int64 PRWord;
typedef unsigned __int64 PRUword;
#else
typedef long PRWord;
typedef unsigned long PRUword;
#endif

#if defined(NO_NSPR_10_SUPPORT)
#else
/********* ???????????????? FIX ME       ??????????????????????????? *****/
/********************** Some old definitions until pr=>ds transition is done ***/
/********************** Also, we are still using NSPR 1.0. GC ******************/

#define PR_PUBLIC_API		PR_IMPLEMENT

#define NSPR_BEGIN_MACRO        do {
#define NSPR_END_MACRO          } while (0)

#ifdef NSPR_BEGIN_EXTERN_C
#undef NSPR_BEGIN_EXTERN_C
#endif
#ifdef NSPR_END_EXTERN_C
#undef NSPR_END_EXTERN_C
#endif

#ifdef __cplusplus
#define NSPR_BEGIN_EXTERN_C     extern "C" {
#define NSPR_END_EXTERN_C       }
#else
#define NSPR_BEGIN_EXTERN_C
#define NSPR_END_EXTERN_C
#endif

/********* ????????????? End Fix me ?????????????????????????????? *****/
#endif /* NO_NSPR_10_SUPPORT */

PR_END_EXTERN_C

#if !defined(NO_NSPR_10_SUPPORT)
#include "base/basictypes.h"
#endif

#endif /* prtypes_h___ */

