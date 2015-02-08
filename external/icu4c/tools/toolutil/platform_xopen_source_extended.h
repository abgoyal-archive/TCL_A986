#ifndef _PLATFORM_XOPEN_SOURCE_EXTENDED_H
#define _PLATFORM_XOPEN_SOURCE_EXTENDED_H

#include "unicode/utypes.h"

#if !defined(_XOPEN_SOURCE_EXTENDED)
#define _XOPEN_SOURCE_EXTENDED 1
#endif

#if (defined(U_AIX) && !defined(__GNUC__)) || (defined(U_SOLARIS) && defined(__GNUC__))
#   if _XOPEN_SOURCE_EXTENDED && !defined(U_HAVE_STD_STRING)
#   define U_HAVE_STD_STRING 0
#   endif
#endif

#endif /* _PLATFORM_XOPEN_SOURCE_EXTENDED_H */
