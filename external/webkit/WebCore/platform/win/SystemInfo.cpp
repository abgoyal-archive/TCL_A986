

#include "config.h"
#include "SystemInfo.h"

#include <windows.h>

namespace WebCore {

bool isRunningOnVistaOrLater()
{
    static bool isVistaOrLater;
    static bool initialized;

    if (initialized)
        return isVistaOrLater;

    initialized = true;

    OSVERSIONINFOEX vi = {0};
    vi.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);
    GetVersionEx(reinterpret_cast<OSVERSIONINFO*>(&vi));

    isVistaOrLater = vi.dwMajorVersion >= 6;

    return isVistaOrLater;
}

} // namespace WebCore
