
#ifndef __WINTZIMPL
#define __WINTZIMPL

#include "unicode/utypes.h"

#ifdef U_WINDOWS
U_CDECL_BEGIN
/* Forward declarations for Windows types... */
typedef struct _TIME_ZONE_INFORMATION TIME_ZONE_INFORMATION;
U_CDECL_END

U_CAPI UBool U_EXPORT2
uprv_getWindowsTimeZoneInfo(TIME_ZONE_INFORMATION *zoneInfo, const UChar *icuid, int32_t length);


#endif /* #ifdef U_WINDOWS */

#endif /* __WINTZIMPL */
