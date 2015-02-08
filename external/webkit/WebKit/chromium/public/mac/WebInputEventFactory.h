

#ifndef WebInputEventFactory_h
#define WebInputEventFactory_h

#include "../WebCommon.h"

#ifdef __OBJC__
@class NSEvent;
@class NSView;
#else
class NSEvent;
class NSView;
#endif

namespace WebKit {

class WebKeyboardEvent;
class WebMouseEvent;
class WebMouseWheelEvent;

class WebInputEventFactory {
public:
    WEBKIT_API static WebKeyboardEvent keyboardEvent(NSEvent*);
    WEBKIT_API static WebKeyboardEvent keyboardEvent(wchar_t character, int modifiers, double timeStampSeconds);
    WEBKIT_API static WebMouseEvent mouseEvent(NSEvent*, NSView*);
    WEBKIT_API static WebMouseWheelEvent mouseWheelEvent(NSEvent*, NSView*);
};

} // namespace WebKit

#endif
