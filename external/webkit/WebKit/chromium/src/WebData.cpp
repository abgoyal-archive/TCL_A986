

#include "config.h"
#include "WebData.h"

#include "SharedBuffer.h"

using namespace WebCore;

namespace WebKit {

class WebDataPrivate : public SharedBuffer {
};

void WebData::reset()
{
    if (m_private) {
        m_private->deref();
        m_private = 0;
    }
}

void WebData::assign(const WebData& other)
{
    WebDataPrivate* p = const_cast<WebDataPrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

void WebData::assign(const char* data, size_t size)
{
    assign(static_cast<WebDataPrivate*>(
        SharedBuffer::create(data, size).releaseRef()));
}

size_t WebData::size() const
{
    if (!m_private)
        return 0;
    return const_cast<WebDataPrivate*>(m_private)->size();
}

const char* WebData::data() const
{
    if (!m_private)
        return 0;
    return const_cast<WebDataPrivate*>(m_private)->data();
}

WebData::WebData(const PassRefPtr<SharedBuffer>& buffer)
    : m_private(static_cast<WebDataPrivate*>(buffer.releaseRef()))
{
}

WebData& WebData::operator=(const PassRefPtr<SharedBuffer>& buffer)
{
    assign(static_cast<WebDataPrivate*>(buffer.releaseRef()));
    return *this;
}

WebData::operator PassRefPtr<SharedBuffer>() const
{
    return PassRefPtr<SharedBuffer>(const_cast<WebDataPrivate*>(m_private));
}

void WebData::assign(WebDataPrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit
