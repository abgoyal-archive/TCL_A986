

#include "Platform.h"

#ifndef ALWAYS_INLINE
#if COMPILER(GCC) && defined(NDEBUG) && !COMPILER(MINGW)
#define ALWAYS_INLINE inline __attribute__((__always_inline__))
#elif (COMPILER(MSVC) || COMPILER(RVCT)) && defined(NDEBUG)
#define ALWAYS_INLINE __forceinline
#else
#define ALWAYS_INLINE inline
#endif
#endif

#ifndef NEVER_INLINE
#if COMPILER(GCC)
#define NEVER_INLINE __attribute__((__noinline__))
#else
#define NEVER_INLINE
#endif
#endif

#ifndef UNLIKELY
#if COMPILER(GCC)
#define UNLIKELY(x) __builtin_expect((x), 0)
#else
#define UNLIKELY(x) (x)
#endif
#endif

#ifndef LIKELY
#if COMPILER(GCC)
#define LIKELY(x) __builtin_expect((x), 1)
#else
#define LIKELY(x) (x)
#endif
#endif

#ifndef NO_RETURN
#if COMPILER(GCC)
#define NO_RETURN __attribute((__noreturn__))
#else
#define NO_RETURN
#endif
#endif
