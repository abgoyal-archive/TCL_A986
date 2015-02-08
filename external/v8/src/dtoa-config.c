


#if !(defined(__APPLE__) && defined(__MACH__)) && \
    !defined(WIN32) && !defined(__FreeBSD__) && !defined(__OpenBSD__) && \
    !defined(__sun)
#include <endian.h>
#endif
#include <math.h>
#include <float.h>

#if !(defined(__APPLE__) && defined(__MACH__)) && !defined(WIN32) && \
    !defined(__FreeBSD__) && !defined(__OpenBSD__) && !defined(__sun) && \
    __FLOAT_WORD_ORDER == __BIG_ENDIAN
#define  IEEE_MC68k
#else
#define  IEEE_8087
#endif

#define __MATH_H__
#if defined(__APPLE__) && defined(__MACH__) || defined(__FreeBSD__) || \
    defined(__OpenBSD__) || defined(__sun)
#include <stdlib.h>

#endif
#if defined(WIN32) && defined(_DLL)
#include "stdlib.h"
#endif

/* For MinGW, turn on __NO_ISOCEXT so that its strtod doesn't get added */
#ifdef __MINGW32__
#define __NO_ISOCEXT
#endif  /* __MINGW32__ */

/* On 64-bit systems, we need to make sure that a Long is only 32 bits. */
#ifdef V8_TARGET_ARCH_X64
#define Long int
#endif /* V8_TARGET_ARCH_X64 */

#define strtod gay_strtod
#include "third_party/dtoa/dtoa.c"
