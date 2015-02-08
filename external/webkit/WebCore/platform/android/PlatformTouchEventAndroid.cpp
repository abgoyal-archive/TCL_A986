

#include "config.h"
#include "PlatformTouchEvent.h"

#if ENABLE(TOUCH_EVENTS)

namespace WebCore {

// These values should be kept in sync with those defined in the android.view.KeyEvent class from the Android SDK.
enum AndroidMetaKeyState {
    META_SHIFT_ON = 0x01,
    META_ALT_ON = 0x02,
    META_SYM_ON = 0x04
};

PlatformTouchEvent::PlatformTouchEvent(const IntPoint& windowPos, TouchEventType type, PlatformTouchPoint::State state, int metaState)
    : m_type(type)
    , m_metaKey(false)
{
    m_touchPoints.append(PlatformTouchPoint(windowPos, state));

    m_altKey = metaState & META_ALT_ON;
    m_shiftKey = metaState & META_SHIFT_ON;
    m_ctrlKey = metaState & META_SYM_ON;
}

}

#endif
