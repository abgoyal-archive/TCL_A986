

#include "config.h"

#if ENABLE(WML)
#include "WMLIntrinsicEventHandler.h"

namespace WebCore {

WMLIntrinsicEventHandler::WMLIntrinsicEventHandler()
{
}

bool WMLIntrinsicEventHandler::registerIntrinsicEvent(WMLIntrinsicEventType type, PassRefPtr<WMLIntrinsicEvent> event)
{
    if (m_events.contains(type))
        return false;

    m_events.set(type, event);
    return true;
}

void WMLIntrinsicEventHandler::deregisterIntrinsicEvent(WMLIntrinsicEventType type)
{
    if (m_events.contains(type))
        m_events.remove(type);
}

void WMLIntrinsicEventHandler::triggerIntrinsicEvent(WMLIntrinsicEventType type) const
{
    RefPtr<WMLIntrinsicEvent> event = m_events.get(type);
    ASSERT(event->taskElement());
    event->taskElement()->executeTask();
}

bool WMLIntrinsicEventHandler::hasIntrinsicEvent(WMLIntrinsicEventType type) const
{
    return m_events.contains(type);
}

}

#endif
