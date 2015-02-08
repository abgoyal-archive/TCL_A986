

#include "config.h"
#include "WebURL.h"

#include "KURL.h"

namespace WebKit {

WebURL::WebURL(const WebCore::KURL& url)
    : m_spec(url.utf8String())
    , m_parsed(url.parsed())
    , m_isValid(url.isValid())
{
}

WebURL& WebURL::operator=(const WebCore::KURL& url)
{
    m_spec = url.utf8String();
    m_parsed = url.parsed();
    m_isValid = url.isValid();
    return *this;
}

WebURL::operator WebCore::KURL() const
{
    return WebCore::KURL(m_spec, m_parsed, m_isValid);
}

} // namespace WebKit
