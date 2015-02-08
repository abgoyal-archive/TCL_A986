

#include "config.h"

#if ENABLE(XBL)

#include "CachedXBLDocument.h"

#include "CachedResourceClientWalker.h"
#include "TextResourceDecoder.h"
#include <wtf/Vector.h>

namespace WebCore {

CachedXBLDocument::CachedXBLDocument(const String &url)
: CachedResource(url, XBL), m_document(0)
{
    // It's XML we want.
    setAccept("text/xml, application/xml, application/xhtml+xml, text/xsl, application/rss+xml, application/atom+xml");

    m_decoder = new TextResourceDecoder("application/xml");
}

CachedXBLDocument::~CachedXBLDocument()
{
    if (m_document)
        m_document->deref();
}

void CachedXBLDocument::ref(CachedResourceClient *c)
{
    CachedResource::ref(c);
    if (!m_loading)
        c->setXBLDocument(m_url, m_document);
}

void CachedXBLDocument::setEncoding(const String& chs)
{
    m_decoder->setEncoding(chs, TextResourceDecoder::EncodingFromHTTPHeader);
}

String CachedXBLDocument::encoding() const
{
    return m_decoder->encoding().name();
}

void CachedXBLDocument::data(Vector<char>& data, bool )
{
    if (!allDataReceived)
        return;
    
    ASSERT(!m_document);
    
    m_document = new XBL::XBLDocument();
    m_document->ref();
    m_document->open();
    
    m_document->write(m_decoder->decode(data.data(), data.size()));
    setSize(data.size());
    
    m_document->finishParsing();
    m_document->close();
    m_loading = false;
    checkNotify();
}

void CachedXBLDocument::checkNotify()
{
    if (m_loading)
        return;
    
    CachedResourceClientWalker w(m_clients);
    while (CachedResourceClient *c = w.next())
        c->setXBLDocument(m_url, m_document);
}

void CachedXBLDocument::error()
{
    m_loading = false;
    m_errorOccurred = true;
    checkNotify();
}

}

#endif
