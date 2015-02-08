

#include "config.h"

#if ENABLE(TOUCH_EVENTS)

#include "TouchEvent.h"

namespace WebCore {

TouchEvent::TouchEvent(TouchList* touches, TouchList* targetTouches,
        TouchList* changedTouches, const AtomicString& type, 
        PassRefPtr<AbstractView> view, int screenX, int screenY, int pageX, int pageY,
        bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
    : MouseRelatedEvent(type, true, true, view, 0, screenX, screenY, pageX, pageY,
                        ctrlKey, altKey, shiftKey, metaKey)
    , m_touches(touches)
    , m_targetTouches(targetTouches)
    , m_changedTouches(changedTouches)
{
}

void TouchEvent::initTouchEvent(TouchList* touches, TouchList* targetTouches,
        TouchList* changedTouches, const AtomicString& type, 
        PassRefPtr<AbstractView> view, int screenX, int screenY, int clientX, int clientY,
        bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
{
    if (dispatched())
        return;

    initUIEvent(type, true, true, view, 0);

    m_screenX = screenX;
    m_screenY = screenY;
    m_ctrlKey = ctrlKey;
    m_altKey = altKey;
    m_shiftKey = shiftKey;
    m_metaKey = metaKey;
    initCoordinates(clientX, clientY);
}

} // namespace WebCore

#endif // ENABLE(TOUCH_EVENTS)
