

#include "config.h"
#include "WebScreenInfoFactory.h"

#include "WebScreenInfo.h"

#include <X11/Xlib.h>

namespace WebKit {

WebScreenInfo WebScreenInfoFactory::screenInfo(Display* display, int screenNumber)
{
    WebScreenInfo results;
    // FIXME: not all screens with use 8bpp.
    results.depthPerComponent = 8;

    int displayWidth = XDisplayWidth(display, screenNumber);
    int displayHeight = XDisplayHeight(display, screenNumber);
    results.depth = XDisplayPlanes(display, screenNumber);
    results.isMonochrome = results.depth == 1;

    results.rect = WebRect(0, 0, displayWidth, displayHeight);

    // I don't know of a way to query the "maximize" size of the window (e.g.
    // screen size less sidebars etc) since this is something which only the
    // window manager knows.
    results.availableRect = results.rect;

    return results;
}

} // namespace WebKit
