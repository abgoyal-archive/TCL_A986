

#ifndef WebScreenInfo_h
#define WebScreenInfo_h

#include "WebRect.h"

namespace WebKit {

struct WebScreenInfo {
    // The screen depth in bits per pixel
    int depth;

    // The bits per colour component. This assumes that the colours are balanced
    // equally.
    int depthPerComponent;

    // This can be true for black and white printers
    bool isMonochrome;

    // This is set from the rcMonitor member of MONITORINFOEX, to whit:
    //   "A RECT structure that specifies the display monitor rectangle,
    //   expressed in virtual-screen coordinates. Note that if the monitor
    //   is not the primary display monitor, some of the rectangle's
    //   coordinates may be negative values."
    WebRect rect;

    // This is set from the rcWork member of MONITORINFOEX, to whit:
    //   "A RECT structure that specifies the work area rectangle of the
    //   display monitor that can be used by applications, expressed in
    //   virtual-screen coordinates. Windows uses this rectangle to
    //   maximize an application on the monitor. The rest of the area in
    //   rcMonitor contains system windows such as the task bar and side
    //   bars. Note that if the monitor is not the primary display monitor,
    //   some of the rectangle's coordinates may be negative values".
    WebRect availableRect;

    WebScreenInfo()
        : depth(0)
        , depthPerComponent(0)
        , isMonochrome(false) { }
};

} // namespace WebKit

#endif
