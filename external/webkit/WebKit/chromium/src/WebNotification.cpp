

#include "config.h"
#include "WebNotification.h"

#if ENABLE(NOTIFICATIONS)

#include "Notification.h"

#include "WebString.h"
#include "WebURL.h"

#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

class WebNotificationPrivate : public Notification {
};

void WebNotification::reset()
{
    assign(0);
}

void WebNotification::assign(const WebNotification& other)
{
    WebNotificationPrivate* p = const_cast<WebNotificationPrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

bool WebNotification::lessThan(const WebNotification& other) const
{
    return reinterpret_cast<uintptr_t>(m_private) < reinterpret_cast<uintptr_t>(other.m_private);
}

bool WebNotification::isHTML() const
{
    return m_private->isHTML();
}

WebURL WebNotification::url() const
{
    ASSERT(isHTML());
    return m_private->url();
}

WebString WebNotification::icon() const
{
    ASSERT(!isHTML());
    return m_private->contents().icon();
}

WebString WebNotification::title() const
{
    ASSERT(!isHTML());
    return m_private->contents().title();
}

WebString WebNotification::body() const
{
    ASSERT(!isHTML());
    return m_private->contents().body();
}

void WebNotification::dispatchDisplayEvent()
{
    RefPtr<Event> event = Event::create("display", false, true);
    m_private->dispatchEvent(event.release());
}

void WebNotification::dispatchErrorEvent(const WebKit::WebString& /* errorMessage */)
{
    // FIXME: errorMessage not supported by WebCore yet
    RefPtr<Event> event = Event::create(eventNames().errorEvent, false, true);
    m_private->dispatchEvent(event.release());
}

void WebNotification::dispatchCloseEvent(bool /* byUser */)
{
    // FIXME: byUser flag not supported by WebCore yet
    RefPtr<Event> event = Event::create(eventNames().closeEvent, false, true);
    m_private->dispatchEvent(event.release());
}

WebNotification::WebNotification(const WTF::PassRefPtr<Notification>& notification)
    : m_private(static_cast<WebNotificationPrivate*>(notification.releaseRef()))
{
}

WebNotification& WebNotification::operator=(const WTF::PassRefPtr<Notification>& notification)
{
    assign(static_cast<WebNotificationPrivate*>(notification.releaseRef()));
    return *this;
}

WebNotification::operator WTF::PassRefPtr<Notification>() const
{
    return WTF::PassRefPtr<Notification>(const_cast<WebNotificationPrivate*>(m_private));
}

void WebNotification::assign(WebNotificationPrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit

#endif // ENABLE(NOTIFICATIONS)
