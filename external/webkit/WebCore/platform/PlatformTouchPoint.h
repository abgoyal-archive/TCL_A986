

#ifndef PlatformTouchPoint_h
#define PlatformTouchPoint_h

#include "IntPoint.h"
#include <wtf/Platform.h>
#include <wtf/Vector.h>

#if ENABLE(TOUCH_EVENTS)

#if PLATFORM(QT)
#include <QTouchEvent>
#endif

namespace WebCore {

class PlatformTouchEvent;

class PlatformTouchPoint {
public:
    enum State {
        TouchReleased,
        TouchPressed,
        TouchMoved,
        TouchStationary,
        TouchCancelled
    };

#if PLATFORM(QT)
    PlatformTouchPoint(const QTouchEvent::TouchPoint&);
#elif PLATFORM(ANDROID)
    PlatformTouchPoint(const IntPoint& windowPos, State);
#endif

    unsigned id() const { return m_id; }
    State state() const { return m_state; }
    IntPoint screenPos() const { return m_screenPos; }
    IntPoint pos() const { return m_pos; }
    
private:
    unsigned m_id;
    State m_state;
    IntPoint m_screenPos;
    IntPoint m_pos;
};

}

#endif // ENABLE(TOUCH_EVENTS)

#endif // PlatformTouchPoint_h
