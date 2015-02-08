


#ifndef __VPXTYPES_H__
#define __VPXTYPES_H__

#include "vpx_ports/config.h"

//#include <sys/types.h>
#ifdef _MSC_VER
# include <basetsd.h>
typedef SSIZE_T ssize_t;
#endif

#if defined(HAVE_STDINT_H) && HAVE_STDINT_H
/* C99 types are preferred to vpx integer types */
# include <stdint.h>
#endif

#if !defined(HAVE_STDINT_H) && !defined(INT_T_DEFINED)
# ifdef STRICTTYPES
typedef signed char  int8_t;
typedef signed short int16_t;
typedef signed int   int32_t;
# else
typedef char         int8_t;
typedef short        int16_t;
typedef int          int32_t;
# endif
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
#endif

typedef int8_t     vpxs8;
typedef uint8_t    vpxu8;
typedef int16_t    vpxs16;
typedef uint16_t   vpxu16;
typedef int32_t    vpxs32;
typedef uint32_t   vpxu32;
typedef int32_t    vpxbool;

enum {vpxfalse, vpxtrue};

#ifdef UNICODE
# ifdef NO_WCHAR
#  error "no non-wchar support added yet"
# else
#  include <wchar.h>
typedef wchar_t vpxtc;
#  define OTC(str) L ## str
#  define VPXTC "ls"
# endif /*NO_WCHAR*/
#else
typedef char vpxtc;
# define OTC(str) (vpxtc*)str
# define VPXTC "s"
#endif /*UNICODE*/
/*@} end - base types*/

#if defined(HAVE_STDINT_H)
# define VPX64 PRId64
typedef int64_t vpxs64;
#elif defined(HASLONGLONG)
# undef  PRId64
# define PRId64 "lld"
# define VPX64 PRId64
typedef long long vpxs64;
#elif defined(WIN32) || defined(_WIN32_WCE)
# undef  PRId64
# define PRId64 "I64d"
# define VPX64 PRId64
typedef __int64 vpxs64;
typedef unsigned __int64 vpxu64;
#elif defined(__uClinux__) && defined(CHIP_DM642)
# include <lddk.h>
# undef  PRId64
# define PRId64 "lld"
# define VPX64 PRId64
typedef long vpxs64;
#elif defined(__SYMBIAN32__)
# undef  PRId64
# define PRId64 "u"
# define VPX64 PRId64
typedef unsigned int vpxs64;
#else
# error "64 bit integer type undefined for this platform!"
#endif
#if !defined(HAVE_STDINT_H) && !defined(INT_T_DEFINED)
typedef vpxs64 int64_t;
typedef vpxu64 uint64_t;
#endif
/*!@} end - base types*/

typedef enum
{
    VPX_NOT_FOUND        = -404,
    VPX_BUFFER_EMPTY     = -202,
    VPX_BUFFER_FULL      = -201,

    VPX_CONNREFUSED      = -102,
    VPX_TIMEDOUT         = -101,
    VPX_WOULDBLOCK       = -100,

    VPX_NET_ERROR        = -9,
    VPX_INVALID_VERSION  = -8,
    VPX_INPROGRESS       = -7,
    VPX_NOT_SUPP         = -6,
    VPX_NO_MEM           = -3,
    VPX_INVALID_PARAMS   = -2,
    VPX_ERROR            = -1,
    VPX_OK               = 0,
    VPX_DONE             = 1
} vpxsc;

#if defined(WIN32) || defined(_WIN32_WCE)
# define DLLIMPORT __declspec(dllimport)
# define DLLEXPORT __declspec(dllexport)
# define DLLLOCAL
#elif defined(LINUX)
# define DLLIMPORT
# if defined(__GNUC__) && ((__GNUC__<<16|(__GNUC_MINOR__&0xff)) >= (3<<16|4))
#  define GCC_HASCLASSVISIBILITY
# endif /*defined(__GNUC__) && __GNUC_PREREQ(3,4)*/
# ifdef GCC_HASCLASSVISIBILITY
#  define DLLEXPORT   __attribute__ ((visibility("default")))
#  define DLLLOCAL __attribute__ ((visibility("hidden")))
# else
#  define DLLEXPORT
#  define DLLLOCAL
# endif /*GCC_HASCLASSVISIBILITY*/
#endif /*platform ifdefs*/

#endif /*__VPXTYPES_H__*/

#undef VPXAPI
#ifdef VPXDLL
# ifdef VPXDLLEXPORT
#  define VPXAPI DLLEXPORT
# else
#  define VPXAPI DLLIMPORT
# endif /*VPXDLLEXPORT*/
#else
# define VPXAPI
#endif /*VPXDLL*/
