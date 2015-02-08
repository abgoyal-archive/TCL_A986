

#include "config.h"
#include "WebEventListenerPrivate.h"

#include "EventListenerWrapper.h"
#include "WebEventListener.h"

namespace WebKit {

WebEventListenerPrivate::WebEventListenerPrivate(WebEventListener* webEventListener)
    : m_webEventListener(webEventListener)
{
}

WebEventListenerPrivate::~WebEventListenerPrivate()
{
}

EventListenerWrapper* WebEventListenerPrivate::createEventListenerWrapper(const WebString& eventType, bool useCapture, Node* node)
{
    EventListenerWrapper* listenerWrapper = new EventListenerWrapper(m_webEventListener);
    WebEventListenerPrivate::ListenerInfo listenerInfo(eventType, useCapture, listenerWrapper, node);
    m_listenerWrappers.append(listenerInfo);
    return listenerWrapper;
}

EventListenerWrapper* WebEventListenerPrivate::getEventListenerWrapper(const WebString& eventType, bool useCapture, Node* node)
{
    Vector<WebEventListenerPrivate::ListenerInfo>::const_iterator iter;
    for (iter = m_listenerWrappers.begin(); iter != m_listenerWrappers.end(); ++iter) {
        if (iter->node == node)
          return iter->eventListenerWrapper;
    }
    ASSERT_NOT_REACHED();
    return 0;
}

void WebEventListenerPrivate::webEventListenerDeleted()
{
    // Notifies all WebEventListenerWrappers that we are going away so they can
    // invalidate their pointer to us.
    Vector<WebEventListenerPrivate::ListenerInfo>::const_iterator iter;
    for (iter = m_listenerWrappers.begin(); iter != m_listenerWrappers.end(); ++iter)
        iter->eventListenerWrapper->webEventListenerDeleted();
}

void WebEventListenerPrivate::eventListenerDeleted(EventListenerWrapper* eventListener)
{
    for (size_t i = 0; i < m_listenerWrappers.size(); ++i) {
        if (m_listenerWrappers[i].eventListenerWrapper == eventListener) {
            m_listenerWrappers.remove(i);
            return;
        }
    }
    ASSERT_NOT_REACHED();
}

} // namespace WebKit
