

#ifndef WebWidget_h
#define WebWidget_h

#include "WebCanvas.h"
#include "WebCommon.h"
#include "WebCompositionCommand.h"
#include "WebTextDirection.h"

namespace WebKit {

class WebInputEvent;
class WebString;
struct WebRect;
struct WebSize;

class WebWidget {
public:
    // This method closes and deletes the WebWidget.
    virtual void close() = 0;

    // Returns the current size of the WebWidget.
    virtual WebSize size() = 0;

    // Called to resize the WebWidget.
    virtual void resize(const WebSize&) = 0;

    // Called to layout the WebWidget.  This MUST be called before Paint,
    // and it may result in calls to WebWidgetClient::didInvalidateRect.
    virtual void layout() = 0;

    // Called to paint the specified region of the WebWidget onto the given
    // canvas.  You MUST call Layout before calling this method.  It is
    // okay to call paint multiple times once layout has been called,
    // assuming no other changes are made to the WebWidget (e.g., once
    // events are processed, it should be assumed that another call to
    // layout is warranted before painting again).
    virtual void paint(WebCanvas*, const WebRect&) = 0;

    // Called to inform the WebWidget of an input event.  Returns true if
    // the event has been processed, false otherwise.
    virtual bool handleInputEvent(const WebInputEvent&) = 0;

    // Called to inform the WebWidget that mouse capture was lost.
    virtual void mouseCaptureLost() = 0;

    // Called to inform the WebWidget that it has gained or lost keyboard focus.
    virtual void setFocus(bool) = 0;

    // Called to inform the WebWidget of a composition event.
    virtual bool handleCompositionEvent(WebCompositionCommand command,
                                        int cursorPosition,
                                        int targetStart,
                                        int targetEnd,
                                        const WebString& text) = 0;

    // Retrieve the status of this WebWidget required by IME APIs.  Upon
    // success enabled and caretBounds are set.
    virtual bool queryCompositionStatus(bool* enabled, WebRect* caretBounds) = 0;

    // Changes the text direction of the selected input node.
    virtual void setTextDirection(WebTextDirection) = 0;

protected:
    ~WebWidget() { }
};

} // namespace WebKit

#endif
