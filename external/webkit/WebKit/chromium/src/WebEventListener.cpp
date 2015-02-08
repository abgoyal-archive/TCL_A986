

#include "config.h"
#include "WebEventListener.h"

#include "WebEventListenerPrivate.h"

namespace WebKit {

WebEventListener::WebEventListener()
    : m_private(new WebEventListenerPrivate(this))
{
}

WebEventListener::~WebEventListener()
{
    m_private->webEventListenerDeleted();
    delete m_private;
}

void WebEventListener::notifyEventListenerDeleted(EventListenerWrapper* wrapper)
{
    m_private->eventListenerDeleted(wrapper);
}

EventListenerWrapper* WebEventListener::createEventListenerWrapper(const WebString& eventType, bool useCapture, Node* node)
{
    return m_private->createEventListenerWrapper(eventType, useCapture, node);
}

EventListenerWrapper* WebEventListener::getEventListenerWrapper(const WebString& eventType, bool useCapture, Node* node)
{
    return m_private->getEventListenerWrapper(eventType, useCapture, node);
}

} // namespace WebKit
