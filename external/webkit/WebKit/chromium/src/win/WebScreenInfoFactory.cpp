

#include "config.h"
#include "WebScreenInfoFactory.h"

#include "WebScreenInfo.h"

#include <windows.h>

namespace WebKit {

static WebRect toWebRect(const RECT& input)
{
    WebRect output;
    output.x = input.left;
    output.y = input.top;
    output.width = input.right - input.left;
    output.height = input.bottom - input.top;
    return output;
}

WebScreenInfo WebScreenInfoFactory::screenInfo(HWND window)
{
    HMONITOR monitor = MonitorFromWindow(window, MONITOR_DEFAULTTOPRIMARY);

    MONITORINFOEX monitorInfo;
    monitorInfo.cbSize = sizeof(MONITORINFOEX);
    GetMonitorInfo(monitor, &monitorInfo);

    DEVMODE devMode;
    devMode.dmSize = sizeof(devMode);
    devMode.dmDriverExtra = 0;
    EnumDisplaySettings(monitorInfo.szDevice, ENUM_CURRENT_SETTINGS, &devMode);

    WebScreenInfo results;
    results.depth = devMode.dmBitsPerPel;
    results.depthPerComponent = devMode.dmBitsPerPel / 3;  // Assumes RGB
    results.isMonochrome = devMode.dmColor == DMCOLOR_MONOCHROME;
    results.rect = toWebRect(monitorInfo.rcMonitor);
    results.availableRect = toWebRect(monitorInfo.rcWork);
    return results;
}

} // namespace WebKit
