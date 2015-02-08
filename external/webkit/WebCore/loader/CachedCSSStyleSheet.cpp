

#include "config.h"
#include "CachedCSSStyleSheet.h"

#include "CachedResourceClient.h"
#include "CachedResourceClientWalker.h"
#include "HTTPParsers.h"
#include "TextResourceDecoder.h"
#include "loader.h"
#include <wtf/Vector.h>

namespace WebCore {

CachedCSSStyleSheet::CachedCSSStyleSheet(const String& url, const String& charset)
    : CachedResource(url, CSSStyleSheet)
    , m_decoder(TextResourceDecoder::create("text/css", charset))
{
    // Prefer text/css but accept any type (dell.com serves a stylesheet
    // as text/html; see <http://bugs.webkit.org/show_bug.cgi?id=11451>).
    setAccept("text/css,*/*;q=0.1");
}

CachedCSSStyleSheet::~CachedCSSStyleSheet()
{
}

void CachedCSSStyleSheet::didAddClient(CachedResourceClient *c)
{
    if (!m_loading)
        c->setCSSStyleSheet(m_url, m_response.url(), m_decoder->encoding().name(), this);
}

void CachedCSSStyleSheet::allClientsRemoved()
{
    if (isSafeToMakePurgeable())
        makePurgeable(true);
}

void CachedCSSStyleSheet::setEncoding(const String& chs)
{
    m_decoder->setEncoding(chs, TextResourceDecoder::EncodingFromHTTPHeader);
}

String CachedCSSStyleSheet::encoding() const
{
    return m_decoder->encoding().name();
}
    
const String CachedCSSStyleSheet::sheetText(bool enforceMIMEType, bool* hasValidMIMEType) const 
{ 
    ASSERT(!isPurgeable());

    if (!m_data || m_data->isEmpty() || !canUseSheet(enforceMIMEType, hasValidMIMEType))
        return String();
    
    if (!m_decodedSheetText.isNull())
        return m_decodedSheetText;
    
    // Don't cache the decoded text, regenerating is cheap and it can use quite a bit of memory
    String sheetText = m_decoder->decode(m_data->data(), m_data->size());
    sheetText += m_decoder->flush();
    return sheetText;
}

void CachedCSSStyleSheet::data(PassRefPtr<SharedBuffer> data, bool allDataReceived)
{
    if (!allDataReceived)
        return;

    m_data = data;
    setEncodedSize(m_data.get() ? m_data->size() : 0);
    // Decode the data to find out the encoding and keep the sheet text around during checkNotify()
    if (m_data) {
        m_decodedSheetText = m_decoder->decode(m_data->data(), m_data->size());
        m_decodedSheetText += m_decoder->flush();
    }
    m_loading = false;
    checkNotify();
    // Clear the decoded text as it is unlikely to be needed immediately again and is cheap to regenerate.
    m_decodedSheetText = String();
}

void CachedCSSStyleSheet::checkNotify()
{
    if (m_loading)
        return;

    CachedResourceClientWalker w(m_clients);
    while (CachedResourceClient *c = w.next())
        c->setCSSStyleSheet(m_url, m_response.url(), m_decoder->encoding().name(), this);
}

void CachedCSSStyleSheet::error()
{
    m_loading = false;
    m_errorOccurred = true;
    checkNotify();
}

bool CachedCSSStyleSheet::canUseSheet(bool enforceMIMEType, bool* hasValidMIMEType) const
{
    if (errorOccurred())
        return false;
        
    if (!enforceMIMEType && !hasValidMIMEType)
        return true;

    // This check exactly matches Firefox.  Note that we grab the Content-Type
    // header directly because we want to see what the value is BEFORE content
    // sniffing.  Firefox does this by setting a "type hint" on the channel.
    // This implementation should be observationally equivalent.
    //
    // This code defaults to allowing the stylesheet for non-HTTP protocols so
    // folks can use standards mode for local HTML documents.
    String mimeType = extractMIMETypeFromMediaType(response().httpHeaderField("Content-Type"));
    bool typeOK = mimeType.isEmpty() || equalIgnoringCase(mimeType, "text/css") || equalIgnoringCase(mimeType, "application/x-unknown-content-type");
    if (hasValidMIMEType)
        *hasValidMIMEType = typeOK;
    if (!enforceMIMEType)
        return true;
    return typeOK;
}
 
}
