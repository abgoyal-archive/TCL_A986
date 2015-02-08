

#include "config.h"
#include "PlatformScreen.h"

#include "HostWindow.h"
#include "IntRect.h"
#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "Page.h"
#include <windows.h>

namespace WebCore {

// Returns info for the default monitor if widget is NULL
static MONITORINFOEX monitorInfoForWidget(Widget* widget)
{
    HWND window = widget ? widget->root()->hostWindow()->platformPageClient() : 0;
    HMONITOR monitor = MonitorFromWindow(window, MONITOR_DEFAULTTOPRIMARY);

    MONITORINFOEX monitorInfo;
    monitorInfo.cbSize = sizeof(MONITORINFOEX);
    GetMonitorInfo(monitor, &monitorInfo);
    return monitorInfo;
}

static DEVMODE deviceInfoForWidget(Widget* widget)
{
    DEVMODE deviceInfo;
    deviceInfo.dmSize = sizeof(DEVMODE);
    deviceInfo.dmDriverExtra = 0;
#if OS(WINCE)
    if (!EnumDisplaySettings(0, ENUM_CURRENT_SETTINGS, &deviceInfo))
        deviceInfo.dmBitsPerPel = 16;
#else
    MONITORINFOEX monitorInfo = monitorInfoForWidget(widget);
    EnumDisplaySettings(monitorInfo.szDevice, ENUM_CURRENT_SETTINGS, &deviceInfo);
#endif

    return deviceInfo;
}

int screenDepth(Widget* widget)
{
    DEVMODE deviceInfo = deviceInfoForWidget(widget);
    return deviceInfo.dmBitsPerPel;
}

int screenDepthPerComponent(Widget* widget)
{
    // FIXME: Assumes RGB -- not sure if this is right.
    DEVMODE deviceInfo = deviceInfoForWidget(widget);
    return deviceInfo.dmBitsPerPel / 3;
}

bool screenIsMonochrome(Widget* widget)
{
#if OS(WINCE)
    // EnumDisplaySettings doesn't set dmColor in DEVMODE.
    return false;
#else
    DEVMODE deviceInfo = deviceInfoForWidget(widget);
    return deviceInfo.dmColor == DMCOLOR_MONOCHROME;
#endif
}

FloatRect screenRect(Widget* widget)
{
    MONITORINFOEX monitorInfo = monitorInfoForWidget(widget);
    return monitorInfo.rcMonitor;
}

FloatRect screenAvailableRect(Widget* widget)
{
    MONITORINFOEX monitorInfo = monitorInfoForWidget(widget);
    return monitorInfo.rcWork;
}

} // namespace WebCore
