

#ifndef WebInputEventFactory_h
#define WebInputEventFactory_h

#include "../WebCommon.h"

typedef struct _GdkEventButton GdkEventButton;
typedef struct _GdkEventMotion GdkEventMotion;
typedef struct _GdkEventCrossing GdkEventCrossing;
typedef struct _GdkEventScroll GdkEventScroll;
typedef struct _GdkEventKey GdkEventKey;

namespace WebKit {

class WebKeyboardEvent;
class WebMouseEvent;
class WebMouseWheelEvent;

class WebInputEventFactory {
public:
    WEBKIT_API static WebKeyboardEvent keyboardEvent(const GdkEventKey*);
    WEBKIT_API static WebKeyboardEvent keyboardEvent(wchar_t character, int state, double timeStampSeconds);
    WEBKIT_API static WebMouseEvent mouseEvent(const GdkEventButton*);
    WEBKIT_API static WebMouseEvent mouseEvent(const GdkEventMotion*);
    WEBKIT_API static WebMouseEvent mouseEvent(const GdkEventCrossing*);
    WEBKIT_API static WebMouseWheelEvent mouseWheelEvent(const GdkEventScroll*);
};

} // namespace WebKit

#endif
