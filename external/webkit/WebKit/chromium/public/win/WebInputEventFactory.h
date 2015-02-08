

#ifndef WebInputEventFactory_h
#define WebInputEventFactory_h

#include "../WebCommon.h"

#include <windows.h>

namespace WebKit {

class WebKeyboardEvent;
class WebMouseEvent;
class WebMouseWheelEvent;

class WebInputEventFactory {
public:
    WEBKIT_API static WebKeyboardEvent keyboardEvent(HWND, UINT, WPARAM, LPARAM);
    WEBKIT_API static WebMouseEvent mouseEvent(HWND, UINT, WPARAM, LPARAM);
    WEBKIT_API static WebMouseWheelEvent mouseWheelEvent(HWND, UINT, WPARAM, LPARAM);

    // Windows only provides information on whether a click was a single or
    // double click, while we need to know the click count past two. The
    // WebInputEventFactory keeps internal state to allow it to synthesize
    // that information. In some cases, like fast-running tests, that
    // information is known to be stale and needs to be reset; that is the
    // function of resetLastClickState().
    WEBKIT_API static void resetLastClickState();
};

} // namespace WebKit

#endif
