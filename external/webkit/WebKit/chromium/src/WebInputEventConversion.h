

#ifndef WebInputEventConversion_h
#define WebInputEventConversion_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebInputEvent.h"

#include "PlatformKeyboardEvent.h"
#include "PlatformMouseEvent.h"
#include "PlatformWheelEvent.h"

namespace WebCore {
class KeyboardEvent;
class MouseEvent;
class ScrollView;
class Widget;
}

namespace WebKit {

class WebMouseEvent;
class WebMouseWheelEvent;
class WebKeyboardEvent;

// These classes are used to convert from WebInputEvent subclasses to
// corresponding WebCore events.

class PlatformMouseEventBuilder : public WebCore::PlatformMouseEvent {
public:
    PlatformMouseEventBuilder(WebCore::Widget*, const WebMouseEvent&);
};

class PlatformWheelEventBuilder : public WebCore::PlatformWheelEvent {
public:
    PlatformWheelEventBuilder(WebCore::Widget*, const WebMouseWheelEvent&);
};

class PlatformKeyboardEventBuilder : public WebCore::PlatformKeyboardEvent {
public:
    PlatformKeyboardEventBuilder(const WebKeyboardEvent&);
    void setKeyType(Type);
    bool isCharacterKey() const;
};

// Converts a WebCore::MouseEvent to a corresponding WebMouseEvent. view is
// the ScrollView corresponding to the event.  Returns true if successful.
// NOTE: This is only implemented for mousemove, mouseover, mouseout,
// mousedown and mouseup.  If the event mapping fails, the event type will
// be set to Undefined.
class WebMouseEventBuilder : public WebMouseEvent {
public:
    WebMouseEventBuilder(const WebCore::ScrollView*, const WebCore::MouseEvent&);
};

// Converts a WebCore::KeyboardEvent to a corresponding WebKeyboardEvent.
// Returns true if successful.  NOTE: This is only implemented for keydown
// and keyup.  If the event mapping fails, the event type will be set to
// Undefined.
class WebKeyboardEventBuilder : public WebKeyboardEvent {
public:
    WebKeyboardEventBuilder(const WebCore::KeyboardEvent&);
};

} // namespace WebKit

#endif
