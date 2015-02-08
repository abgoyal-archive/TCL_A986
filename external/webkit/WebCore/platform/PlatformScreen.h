

#ifndef PlatformScreen_h
#define PlatformScreen_h

#include "FloatRect.h"
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

#if PLATFORM(MAC)
#ifdef __OBJC__
    @class NSScreen;
    @class NSWindow;
#else
    class NSScreen;
    class NSWindow;
#endif
#endif

namespace WebCore {

    class FloatRect;
    class Widget;

    int screenDepth(Widget*);
    int screenDepthPerComponent(Widget*);
    bool screenIsMonochrome(Widget*);

    FloatRect screenRect(Widget*);
    FloatRect screenAvailableRect(Widget*);

#if PLATFORM(MAC)
    NSScreen *screenForWindow(NSWindow *);

    FloatRect toUserSpace(const NSRect&, NSWindow *destination);
    NSRect toDeviceSpace(const FloatRect&, NSWindow *source);

    NSPoint flipScreenPoint(const NSPoint&, NSScreen *);
#endif

} // namespace WebCore

#endif // PlatformScreen_h
