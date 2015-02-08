

#include "config.h"
#include "CachedXSLStyleSheet.h"

#include "CachedResourceClient.h"
#include "CachedResourceClientWalker.h"
#include "TextResourceDecoder.h"
#include <wtf/Vector.h>

namespace WebCore {

#if ENABLE(XSLT)

CachedXSLStyleSheet::CachedXSLStyleSheet(const String &url)
    : CachedResource(url, XSLStyleSheet)
    , m_decoder(TextResourceDecoder::create("text/xsl"))
{
    // It's XML we want.
    // FIXME: This should accept more general xml formats */*+xml, image/svg+xml for example.
    setAccept("text/xml, application/xml, application/xhtml+xml, text/xsl, application/rss+xml, application/atom+xml");
}

void CachedXSLStyleSheet::didAddClient(CachedResourceClient* c)
{  
    if (!m_loading)
        c->setXSLStyleSheet(m_url, m_response.url(), m_sheet);
}

void CachedXSLStyleSheet::setEncoding(const String& chs)
{
    m_decoder->setEncoding(chs, TextResourceDecoder::EncodingFromHTTPHeader);
}

String CachedXSLStyleSheet::encoding() const
{
    return m_decoder->encoding().name();
}

void CachedXSLStyleSheet::data(PassRefPtr<SharedBuffer> data, bool allDataReceived)
{
    if (!allDataReceived)
        return;

    m_data = data;     
    setEncodedSize(m_data.get() ? m_data->size() : 0);
    if (m_data.get()) {
        m_sheet = String(m_decoder->decode(m_data->data(), encodedSize()));
        m_sheet += m_decoder->flush();
    }
    m_loading = false;
    checkNotify();
}

void CachedXSLStyleSheet::checkNotify()
{
    if (m_loading)
        return;
    
    CachedResourceClientWalker w(m_clients);
    while (CachedResourceClient *c = w.next())
        c->setXSLStyleSheet(m_url, m_response.url(), m_sheet);
}

void CachedXSLStyleSheet::error()
{
    m_loading = false;
    m_errorOccurred = true;
    checkNotify();
}

#endif

}
