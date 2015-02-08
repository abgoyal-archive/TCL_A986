

#include "config.h"
#include "SystemTime.h"

#include <windows.h>

#if COMPILER(MINGW) || (PLATFORM(QT) && COMPILER(MSVC))
#include <float.h>
#define FLOAT_MAX FLT_MAX
#endif

namespace WebCore {

float userIdleTime()
{
#if !OS(WINCE)
    LASTINPUTINFO lastInputInfo = {0};
    lastInputInfo.cbSize = sizeof(LASTINPUTINFO);
    if (::GetLastInputInfo(&lastInputInfo))
        return (GetTickCount() - lastInputInfo.dwTime) * 0.001; // ::GetTickCount returns ms of uptime valid for up to 49.7 days.
#endif
    return FLT_MAX; // return an arbitrarily high userIdleTime so that releasing pages from the page cache isn't postponed. 
}

}
