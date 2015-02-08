

#include "config.h"
#include "WebCString.h"

#include "CString.h"
#include "TextEncoding.h"

#include "WebString.h"

namespace WebKit {

class WebCStringPrivate : public WebCore::CStringBuffer {
};

void WebCString::reset()
{
    if (m_private) {
        m_private->deref();
        m_private = 0;
    }
}

void WebCString::assign(const WebCString& other)
{
    assign(const_cast<WebCStringPrivate*>(other.m_private));
}

void WebCString::assign(const char* data, size_t length)
{
    char* newData;
    RefPtr<WebCore::CStringBuffer> buffer =
        WebCore::CString::newUninitialized(length, newData).buffer();
    memcpy(newData, data, length);
    assign(static_cast<WebCStringPrivate*>(buffer.get()));
}

size_t WebCString::length() const
{
    if (!m_private)
        return 0;
    // NOTE: The buffer's length includes the null byte.
    return const_cast<WebCStringPrivate*>(m_private)->length() - 1;
}

const char* WebCString::data() const
{
    if (!m_private)
        return 0;
    return const_cast<WebCStringPrivate*>(m_private)->data();
}

WebString WebCString::utf16() const
{
    return WebCore::UTF8Encoding().decode(data(), length());
}

WebCString WebCString::fromUTF16(const WebUChar* data, size_t length)
{
    return WebCore::UTF8Encoding().encode(
        data, length, WebCore::QuestionMarksForUnencodables);
}

WebCString WebCString::fromUTF16(const WebUChar* data)
{
    size_t len = 0;
    while (data[len] != WebUChar(0))
        len++;
    return fromUTF16(data, len);
}

WebCString::WebCString(const WebCore::CString& s)
    : m_private(static_cast<WebCStringPrivate*>(s.buffer()))
{
    if (m_private)
        m_private->ref();
}

WebCString& WebCString::operator=(const WebCore::CString& s)
{
    assign(static_cast<WebCStringPrivate*>(s.buffer()));
    return *this;
}

WebCString::operator WebCore::CString() const
{
    return m_private;
}

void WebCString::assign(WebCStringPrivate* p)
{
    // Take care to handle the case where m_private == p
    if (p)
        p->ref();
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit
