

#include "config.h"
#include "WebEvent.h"

#include "Event.h"
#include "Node.h"
#include <wtf/PassRefPtr.h>

namespace WebKit {

class WebEventPrivate : public WebCore::Event {
};

void WebEvent::reset()
{
    assign(0);
}

void WebEvent::assign(const WebEvent& other)
{
    WebEventPrivate* p = const_cast<WebEventPrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

void WebEvent::assign(WebEventPrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

WebEvent::WebEvent(const WTF::PassRefPtr<WebCore::Event>& event)
    : m_private(static_cast<WebEventPrivate*>(event.releaseRef()))
{
}

WebString WebEvent::type() const
{
    ASSERT(m_private);
    return m_private->type();
}

WebNode WebEvent::target() const
{
    ASSERT(m_private);
    return WebNode(m_private->target()->toNode());
}

WebNode WebEvent::currentTarget() const
{
    ASSERT(m_private);
    return WebNode(m_private->currentTarget()->toNode());
}

WebEvent::PhaseType WebEvent::eventPhase() const
{
    ASSERT(m_private);
    return static_cast<WebEvent::PhaseType>(m_private->eventPhase());
}

bool WebEvent::bubbles() const
{
    ASSERT(m_private);
    return m_private->bubbles();
}

bool WebEvent::cancelable() const
{
    ASSERT(m_private);
    return m_private->cancelable();
}

bool WebEvent::isUIEvent() const
{
    ASSERT(m_private);
    return m_private->isUIEvent();
}

bool WebEvent::isMouseEvent() const
{
    ASSERT(m_private);
    return m_private->isMouseEvent();
}

bool WebEvent::isMutationEvent() const
{
    ASSERT(m_private);
    return m_private->isMutationEvent();
}

bool WebEvent::isKeyboardEvent() const
{
    ASSERT(m_private);
    return m_private->isKeyboardEvent();
}

bool WebEvent::isTextEvent() const
{
    ASSERT(m_private);
    return m_private->isTextEvent();
}

bool WebEvent::isCompositionEvent() const
{
    ASSERT(m_private);
    return m_private->isCompositionEvent();
}

bool WebEvent::isDragEvent() const
{
    ASSERT(m_private);
    return m_private->isDragEvent();
}

bool WebEvent::isClipboardEvent() const
{
    ASSERT(m_private);
    return m_private->isClipboardEvent();
}

bool WebEvent::isMessageEvent() const
{
    ASSERT(m_private);
    return m_private->isMessageEvent();
}

bool WebEvent::isWheelEvent() const
{
    ASSERT(m_private);
    return m_private->isWheelEvent();
}

bool WebEvent::isBeforeTextInsertedEvent() const
{
    ASSERT(m_private);
    return m_private->isBeforeTextInsertedEvent();
}

bool WebEvent::isOverflowEvent() const
{
    ASSERT(m_private);
    return m_private->isOverflowEvent();
}

bool WebEvent::isPageTransitionEvent() const
{
    ASSERT(m_private);
    return m_private->isPageTransitionEvent();
}

bool WebEvent::isPopStateEvent() const
{
    ASSERT(m_private);
    return m_private->isPopStateEvent();
}

bool WebEvent::isProgressEvent() const
{
    ASSERT(m_private);
    return m_private->isProgressEvent();
}

bool WebEvent::isXMLHttpRequestProgressEvent() const
{
    ASSERT(m_private);
    return m_private->isXMLHttpRequestProgressEvent();
}

bool WebEvent::isWebKitAnimationEvent() const
{
    ASSERT(m_private);
    return m_private->isWebKitAnimationEvent();
}

bool WebEvent::isWebKitTransitionEvent() const
{
    ASSERT(m_private);
    return m_private->isWebKitTransitionEvent();
}

bool WebEvent::isBeforeLoadEvent() const
{
    ASSERT(m_private);
    return m_private->isBeforeLoadEvent();
}

} // namespace WebKit
