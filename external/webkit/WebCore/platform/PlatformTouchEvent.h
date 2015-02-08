

#ifndef PlatformTouchEvent_h
#define PlatformTouchEvent_h

#include "PlatformTouchPoint.h"
#include <wtf/Vector.h>

#if ENABLE(TOUCH_EVENTS)

#if PLATFORM(QT)
QT_BEGIN_NAMESPACE
class QTouchEvent;
QT_END_NAMESPACE
#endif

#if PLATFORM(ANDROID)
#include "IntPoint.h"
#endif

namespace WebCore {

enum TouchEventType {
    TouchStart
    , TouchMove
    , TouchEnd
    , TouchCancel
#if PLATFORM(ANDROID)
    , TouchLongPress
    , TouchDoubleTap
#endif
};

class PlatformTouchEvent {
public:
    PlatformTouchEvent()
        : m_type(TouchStart)
        , m_ctrlKey(false)
        , m_altKey(false)
        , m_shiftKey(false)
        , m_metaKey(false)
    {}
#if PLATFORM(QT)
    PlatformTouchEvent(QTouchEvent*);
#elif PLATFORM(ANDROID)
    PlatformTouchEvent(const IntPoint& windowPos, TouchEventType, PlatformTouchPoint::State, int metaState);
#endif

    TouchEventType type() const { return m_type; }
    const Vector<PlatformTouchPoint>& touchPoints() const { return m_touchPoints; }

    bool ctrlKey() const { return m_ctrlKey; }
    bool altKey() const { return m_altKey; }
    bool shiftKey() const { return m_shiftKey; }
    bool metaKey() const { return m_metaKey; }

private:
    TouchEventType m_type;
    Vector<PlatformTouchPoint> m_touchPoints;
    bool m_ctrlKey;
    bool m_altKey;
    bool m_shiftKey;
    bool m_metaKey;
};

}

#endif // ENABLE(TOUCH_EVENTS)

#endif // PlatformTouchEvent_h
