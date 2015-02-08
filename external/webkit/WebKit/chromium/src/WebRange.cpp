

#include "config.h"
#include "WebRange.h"

#include "Range.h"
#include "WebNode.h"
#include "WebString.h"
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

class WebRangePrivate : public Range {
};

void WebRange::reset()
{
    assign(0);
}

void WebRange::assign(const WebRange& other)
{
    WebRangePrivate* p = const_cast<WebRangePrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

int WebRange::startOffset() const
{
    return m_private->startOffset();
}

int WebRange::endOffset() const
{
    return m_private->endOffset();
}

WebNode WebRange::startContainer(int& exceptionCode) const
{
    return PassRefPtr<Node>(m_private->startContainer(exceptionCode));
}

WebNode WebRange::endContainer(int& exceptionCode) const
{
    return PassRefPtr<Node>(m_private->endContainer(exceptionCode));
}

WebString WebRange::toHTMLText() const
{
    return m_private->toHTML();
}

WebString WebRange::toPlainText() const
{
    return m_private->text();
}

WebRange::WebRange(const WTF::PassRefPtr<WebCore::Range>& range)
    : m_private(static_cast<WebRangePrivate*>(range.releaseRef()))
{
}

WebRange& WebRange::operator=(const WTF::PassRefPtr<WebCore::Range>& range)
{
    assign(static_cast<WebRangePrivate*>(range.releaseRef()));
    return *this;
}

WebRange::operator WTF::PassRefPtr<WebCore::Range>() const
{
    return PassRefPtr<Range>(const_cast<WebRangePrivate*>(m_private));
}

void WebRange::assign(WebRangePrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

} // namespace WebKit
