

#include "config.h"
#include "EventListenerWrapper.h"

#include "Event.h"
#include "EventListener.h"

#include "WebEvent.h"
#include "WebEventListener.h"

namespace WebKit {

EventListenerWrapper::EventListenerWrapper(WebEventListener* webEventListener)
    : EventListener(EventListener::JSEventListenerType)
    , m_webEventListener(webEventListener)
{
}

EventListenerWrapper::~EventListenerWrapper()
{
    if (m_webEventListener)
        m_webEventListener->notifyEventListenerDeleted(this);
}

bool EventListenerWrapper::operator==(const EventListener& listener)
{
    return this == &listener;
}

void EventListenerWrapper::handleEvent(ScriptExecutionContext* context, Event* event)
{
    if (!m_webEventListener)
        return;
    WebEvent webEvent(event);
    m_webEventListener->handleEvent(webEvent);
}

void EventListenerWrapper::webEventListenerDeleted()
{
    m_webEventListener = 0;
}

} // namespace WebKit
