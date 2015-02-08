

#include "config.h"
#include "PlatformTouchPoint.h"

#if ENABLE(TOUCH_EVENTS)

namespace WebCore {

PlatformTouchPoint::PlatformTouchPoint(const QTouchEvent::TouchPoint& point)
{
    // The QTouchEvent::TouchPoint API states that ids will be >= 0.
    m_id = static_cast<unsigned>(point.id());
    switch (point.state()) {
    case Qt::TouchPointReleased: m_state = TouchReleased; break;
    case Qt::TouchPointMoved: m_state = TouchMoved; break;
    case Qt::TouchPointPressed: m_state = TouchPressed; break;
    case Qt::TouchPointStationary: m_state = TouchStationary; break;
    }
    m_screenPos = point.screenPos().toPoint();
    m_pos = point.pos().toPoint();
}

}

#endif
