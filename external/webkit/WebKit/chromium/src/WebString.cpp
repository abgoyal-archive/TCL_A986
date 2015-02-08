

#include "config.h"
#include "WebString.h"

#include "AtomicString.h"
#include "CString.h"
#include "PlatformString.h"

#include "WebCString.h"

namespace WebKit {

class WebStringPrivate : public WebCore::StringImpl {
};

void WebString::reset()
{
    if (m_private) {
        m_private->deref();
        m_private = 0;
    }
}

void WebString::assign(const WebString& other)
{
    assign(const_cast<WebStringPrivate*>(other.m_private));
}

void WebString::assign(const WebUChar* data, size_t length)
{
    assign(static_cast<WebStringPrivate*>(
        WebCore::StringImpl::create(data, length).get()));
}

size_t WebString::length() const
{
    return m_private ? const_cast<WebStringPrivate*>(m_private)->length() : 0;
}

const WebUChar* WebString::data() const
{
    return m_private ? const_cast<WebStringPrivate*>(m_private)->characters() : 0;
}

WebCString WebString::utf8() const
{
    return WebCore::String(m_private).utf8();
}

WebString WebString::fromUTF8(const char* data, size_t length)
{
    return WebCore::String::fromUTF8(data, length);
}

WebString WebString::fromUTF8(const char* data)
{
    return WebCore::String::fromUTF8(data);
}

bool WebString::equals(const WebString& s) const
{
    return equal(m_private, s.m_private);
}

WebString::WebString(const WebCore::String& s)
    : m_private(static_cast<WebStringPrivate*>(s.impl()))
{
    if (m_private)
        m_private->ref();
}

WebString& WebString::operator=(const WebCore::String& s)
{
    assign(static_cast<WebStringPrivate*>(s.impl()));
    return *this;
}

WebString::operator WebCore::String() const
{
    return m_private;
}

WebString::WebString(const WebCore::AtomicString& s)
    : m_private(0)
{
    assign(s.string());
}

WebString& WebString::operator=(const WebCore::AtomicString& s)
{
    assign(s.string());
    return *this;
}

WebString::operator WebCore::AtomicString() const
{
    return WebCore::AtomicString(static_cast<WebCore::StringImpl *>(m_private));
}

void WebString::assign(WebStringPrivate* p)
{
    // Take care to handle the case where m_private == p
    if (p)
        p->ref();
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit
