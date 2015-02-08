

#ifndef WebWidgetClient_h
#define WebWidgetClient_h

#include "WebCommon.h"
#include "WebNavigationPolicy.h"
#include "WebRect.h"
#include "WebScreenInfo.h"

namespace WebKit {

class WebWidget;
struct WebCursorInfo;

class WebWidgetClient {
public:
    // Called when a region of the WebWidget needs to be re-painted.
    virtual void didInvalidateRect(const WebRect&) { }

    // Called when a region of the WebWidget, given by clipRect, should be
    // scrolled by the specified dx and dy amounts.
    virtual void didScrollRect(int dx, int dy, const WebRect& clipRect) { }

    // Called when the widget acquires or loses focus, respectively.
    virtual void didFocus() { }
    virtual void didBlur() { }

    // Called when the cursor for the widget changes.
    virtual void didChangeCursor(const WebCursorInfo&) { }

    // Called when the widget should be closed.  WebWidget::close() should
    // be called asynchronously as a result of this notification.
    virtual void closeWidgetSoon() { }

    // Called to show the widget according to the given policy.
    virtual void show(WebNavigationPolicy) { }

    // Called to block execution of the current thread until the widget is
    // closed.
    virtual void runModal() { }

    // Called to get/set the position of the widget in screen coordinates.
    virtual WebRect windowRect() { return WebRect(); }
    virtual void setWindowRect(const WebRect&) { }

    // Called to get the position of the resizer rect in window coordinates.
    virtual WebRect windowResizerRect() { return WebRect(); }

    // Called to get the position of the root window containing the widget
    // in screen coordinates.
    virtual WebRect rootWindowRect() { return WebRect(); }

    // Called to query information about the screen where this widget is
    // displayed.
    virtual WebScreenInfo screenInfo() { return WebScreenInfo(); }

protected:
    ~WebWidgetClient() { }
};

} // namespace WebKit

#endif
