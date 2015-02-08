

#include "config.h"
#include "WheelEvent.h"

#include "EventNames.h"
#include <wtf/MathExtras.h>

namespace WebCore {

WheelEvent::WheelEvent()
    : m_wheelDeltaX(0)
    , m_wheelDeltaY(0)
{
}

WheelEvent::WheelEvent(float wheelTicksX, float wheelTicksY, PassRefPtr<AbstractView> view,
                       int screenX, int screenY, int pageX, int pageY,
                       bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
    : MouseRelatedEvent(eventNames().mousewheelEvent,
                        true, true, view, 0, screenX, screenY, pageX, pageY, 
                        ctrlKey, altKey, shiftKey, metaKey)
    , m_wheelDeltaX(lroundf(wheelTicksX * 120))
    , m_wheelDeltaY(lroundf(wheelTicksY * 120)) // Normalize to the Windows 120 multiple
{
}

void WheelEvent::initWheelEvent(int wheelDeltaX, int wheelDeltaY, PassRefPtr<AbstractView> view,
                                int screenX, int screenY, int pageX, int pageY,
                                bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
{
    if (dispatched())
        return;
    
    initUIEvent(eventNames().mousewheelEvent, true, true, view, 0);
    
    m_screenX = screenX;
    m_screenY = screenY;
    m_ctrlKey = ctrlKey;
    m_altKey = altKey;
    m_shiftKey = shiftKey;
    m_metaKey = metaKey;
    m_wheelDeltaX = wheelDeltaX;
    m_wheelDeltaY = wheelDeltaY;
    
    initCoordinates(pageX, pageY);
}


bool WheelEvent::isWheelEvent() const
{
    return true;
}

} // namespace WebCore
