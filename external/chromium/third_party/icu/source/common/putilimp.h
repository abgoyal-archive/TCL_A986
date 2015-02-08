
#ifndef PUTILIMP_H
#define PUTILIMP_H

#include "unicode/utypes.h"
#include "unicode/putil.h"

/*==========================================================================*/
/* Platform utilities                                                       */
/*==========================================================================*/


U_INTERNAL UBool   U_EXPORT2 uprv_isNaN(double d);
U_INTERNAL UBool   U_EXPORT2 uprv_isInfinite(double d);
U_INTERNAL UBool   U_EXPORT2 uprv_isPositiveInfinity(double d);
U_INTERNAL UBool   U_EXPORT2 uprv_isNegativeInfinity(double d);
U_INTERNAL double  U_EXPORT2 uprv_getNaN(void);
U_INTERNAL double  U_EXPORT2 uprv_getInfinity(void);

U_INTERNAL double  U_EXPORT2 uprv_trunc(double d);
U_INTERNAL double  U_EXPORT2 uprv_floor(double d);
U_INTERNAL double  U_EXPORT2 uprv_ceil(double d);
U_INTERNAL double  U_EXPORT2 uprv_fabs(double d);
U_INTERNAL double  U_EXPORT2 uprv_modf(double d, double* pinteger);
U_INTERNAL double  U_EXPORT2 uprv_fmod(double d, double y);
U_INTERNAL double  U_EXPORT2 uprv_pow(double d, double exponent);
U_INTERNAL double  U_EXPORT2 uprv_pow10(int32_t exponent);
U_INTERNAL double  U_EXPORT2 uprv_fmax(double d, double y);
U_INTERNAL double  U_EXPORT2 uprv_fmin(double d, double y);
U_INTERNAL int32_t U_EXPORT2 uprv_max(int32_t d, int32_t y);
U_INTERNAL int32_t U_EXPORT2 uprv_min(int32_t d, int32_t y);

#if U_IS_BIG_ENDIAN
#   define uprv_isNegative(number) (*((signed char *)&(number))<0)
#else
#   define uprv_isNegative(number) (*((signed char *)&(number)+sizeof(number)-1)<0)
#endif

U_INTERNAL double  U_EXPORT2 uprv_maxMantissa(void);

U_INTERNAL double  U_EXPORT2 uprv_log(double d);

U_INTERNAL double  U_EXPORT2 uprv_round(double x);

#if 0
/*U_INTERNAL int32_t  U_EXPORT2 uprv_digitsAfterDecimal(double x);*/
#endif

U_INTERNAL void     U_EXPORT2 uprv_tzset(void);

U_INTERNAL int32_t  U_EXPORT2 uprv_timezone(void);

U_INTERNAL const char* U_EXPORT2 uprv_tzname(int n);

U_INTERNAL UDate U_EXPORT2 uprv_getUTCtime(void);

U_INTERNAL UBool U_EXPORT2 uprv_pathIsAbsolute(const char *path);

U_INTERNAL void * U_EXPORT2 uprv_maximumPtr(void *base);

#ifndef U_MAX_PTR
#  if defined(OS390) && !defined(_LP64)
    /* We have 31-bit pointers. */
#    define U_MAX_PTR(base) ((void *)0x7fffffff)
#  elif defined(OS400)
#    define U_MAX_PTR(base) uprv_maximumPtr((void *)base)
#  elif defined(__GNUC__) && __GNUC__ >= 4
#    define U_MAX_PTR(base) \
    ((void *)(((uintptr_t)(base)+0x7fffffffu) > (uintptr_t)(base) \
        ? ((uintptr_t)(base)+0x7fffffffu) \
        : (uintptr_t)-1))
#  else
#    define U_MAX_PTR(base) \
    ((char *)(((char *)(base)+0x7fffffffu) > (char *)(base) \
        ? ((char *)(base)+0x7fffffffu) \
        : (char *)-1))
#  endif
#endif

#endif
