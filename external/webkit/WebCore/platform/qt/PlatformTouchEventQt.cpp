

#include "config.h"
#include "PlatformTouchEvent.h"

#if ENABLE(TOUCH_EVENTS)

namespace WebCore {

PlatformTouchEvent::PlatformTouchEvent(QTouchEvent* event)
{
    switch (event->type()) {
    case QEvent::TouchBegin: m_type = TouchStart; break;
    case QEvent::TouchUpdate: m_type = TouchMove; break;
    case QEvent::TouchEnd: m_type = TouchEnd; break;
    }
    const QList<QTouchEvent::TouchPoint>& points = event->touchPoints();
    for (int i = 0; i < points.count(); ++i)
        m_touchPoints.append(PlatformTouchPoint(points.at(i)));

    m_ctrlKey = (event->modifiers() & Qt::ControlModifier);
    m_altKey = (event->modifiers() & Qt::AltModifier);
    m_shiftKey = (event->modifiers() & Qt::ShiftModifier);
    m_metaKey = (event->modifiers() & Qt::MetaModifier);
}

}

#endif
