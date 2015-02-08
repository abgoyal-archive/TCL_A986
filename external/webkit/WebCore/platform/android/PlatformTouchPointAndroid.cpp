

#include "config.h"
#include "PlatformTouchPoint.h"

#if ENABLE(TOUCH_EVENTS)

namespace WebCore {

PlatformTouchPoint::PlatformTouchPoint(const IntPoint& windowPos, State state)
    : m_id(0)
    , m_state(state)
    , m_screenPos(windowPos)
    , m_pos(windowPos) { }

}

#endif
