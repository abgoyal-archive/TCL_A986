

#include "config.h"
#include "WebSecurityOrigin.h"

#include "SecurityOrigin.h"
#include "WebString.h"
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

class WebSecurityOriginPrivate : public SecurityOrigin {
};

WebSecurityOrigin* WebSecurityOrigin::createFromDatabaseIdentifier(const WebString& databaseIdentifier)
{
    return new WebSecurityOrigin(SecurityOrigin::createFromDatabaseIdentifier(databaseIdentifier));
}

WebSecurityOrigin WebSecurityOrigin::createFromString(const WebString& origin)
{
    return WebSecurityOrigin(SecurityOrigin::createFromString(origin));
}

void WebSecurityOrigin::reset()
{
    assign(0);
}

void WebSecurityOrigin::assign(const WebSecurityOrigin& other)
{
    WebSecurityOriginPrivate* p = const_cast<WebSecurityOriginPrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

WebString WebSecurityOrigin::protocol() const
{
    ASSERT(m_private);
    return m_private->protocol();
}

WebString WebSecurityOrigin::host() const
{
    ASSERT(m_private);
    return m_private->host();
}

unsigned short WebSecurityOrigin::port() const
{
    ASSERT(m_private);
    return m_private->port();
}

bool WebSecurityOrigin::isEmpty() const
{
    ASSERT(m_private);
    return m_private->isEmpty();
}

WebString WebSecurityOrigin::toString() const
{
    ASSERT(m_private);
    return m_private->toString();
}

WebString WebSecurityOrigin::databaseIdentifier()
{
    ASSERT(m_private);
    return m_private->databaseIdentifier();
}

WebSecurityOrigin::WebSecurityOrigin(const WTF::PassRefPtr<WebCore::SecurityOrigin>& origin)
    : m_private(static_cast<WebSecurityOriginPrivate*>(origin.releaseRef()))
{
}

WebSecurityOrigin& WebSecurityOrigin::operator=(const WTF::PassRefPtr<WebCore::SecurityOrigin>& origin)
{
    assign(static_cast<WebSecurityOriginPrivate*>(origin.releaseRef()));
    return *this;
}

WebSecurityOrigin::operator WTF::PassRefPtr<WebCore::SecurityOrigin>() const
{
    return PassRefPtr<SecurityOrigin>(const_cast<WebSecurityOriginPrivate*>(m_private));
}

void WebSecurityOrigin::assign(WebSecurityOriginPrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit
