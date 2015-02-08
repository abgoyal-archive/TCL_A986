
#ifndef __WINTZ
#define __WINTZ

#include "unicode/utypes.h"

#ifdef U_WINDOWS


U_CDECL_BEGIN
/* Forward declarations for Windows types... */
typedef struct _TIME_ZONE_INFORMATION TIME_ZONE_INFORMATION;
U_CDECL_END

U_CFUNC const char* U_EXPORT2
uprv_detectWindowsTimeZone();

#endif /* #ifdef U_WINDOWS */

#endif /* __WINTZ */
