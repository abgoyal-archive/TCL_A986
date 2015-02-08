

#include "config.h"
#include "CachedFont.h"

#if PLATFORM(CG) || PLATFORM(QT) || PLATFORM(GTK) || (PLATFORM(CHROMIUM) && (OS(WINDOWS) || OS(LINUX))) || PLATFORM(HAIKU) || OS(WINCE) || OS(ANDROID)
#define STORE_FONT_CUSTOM_PLATFORM_DATA
#endif

#include "Cache.h"
#include "CachedResourceClient.h"
#include "CachedResourceClientWalker.h"
#include "DOMImplementation.h"
#include "FontPlatformData.h"
#ifdef STORE_FONT_CUSTOM_PLATFORM_DATA
#include "FontCustomPlatformData.h"
#endif
#include "TextResourceDecoder.h"
#include "loader.h"
#include <wtf/Vector.h>

#if ENABLE(SVG_FONTS)
#include "HTMLNames.h"
#include "NodeList.h"
#include "SVGElement.h"
#include "SVGFontElement.h"
#include "SVGGElement.h"
#endif

namespace WebCore {

CachedFont::CachedFont(const String &url)
    : CachedResource(url, FontResource)
    , m_fontData(0)
    , m_loadInitiated(false)
#if ENABLE(SVG_FONTS)
    , m_isSVGFont(false)
#endif
{
}

CachedFont::~CachedFont()
{
#ifdef STORE_FONT_CUSTOM_PLATFORM_DATA
    delete m_fontData;
#endif
}

void CachedFont::load(DocLoader*)
{
    // Don't load the file yet.  Wait for an access before triggering the load.
    m_loading = true;
}

void CachedFont::didAddClient(CachedResourceClient* c)
{
    if (!m_loading)
        c->fontLoaded(this);
}

void CachedFont::data(PassRefPtr<SharedBuffer> data, bool allDataReceived)
{
    if (!allDataReceived)
        return;

    m_data = data;     
    setEncodedSize(m_data.get() ? m_data->size() : 0);
    m_loading = false;
    checkNotify();
}

void CachedFont::beginLoadIfNeeded(DocLoader* dl)
{
    if (!m_loadInitiated) {
        m_loadInitiated = true;
        cache()->loader()->load(dl, this, false);
    }
}

bool CachedFont::ensureCustomFontData()
{
#ifdef STORE_FONT_CUSTOM_PLATFORM_DATA
#if ENABLE(SVG_FONTS)
    ASSERT(!m_isSVGFont);
#endif
    if (!m_fontData && !m_errorOccurred && !m_loading && m_data) {
        m_fontData = createFontCustomPlatformData(m_data.get());
        if (!m_fontData)
            m_errorOccurred = true;
    }
#endif
    return m_fontData;
}

FontPlatformData CachedFont::platformDataFromCustomData(float size, bool bold, bool italic, FontRenderingMode renderingMode)
{
#if ENABLE(SVG_FONTS)
    if (m_externalSVGDocument)
        return FontPlatformData(size, bold, italic);
#endif
#ifdef STORE_FONT_CUSTOM_PLATFORM_DATA
    ASSERT(m_fontData);
    return m_fontData->fontPlatformData(static_cast<int>(size), bold, italic, renderingMode);
#else
    return FontPlatformData();
#endif
}

#if ENABLE(SVG_FONTS)
bool CachedFont::ensureSVGFontData()
{
    ASSERT(m_isSVGFont);
    if (!m_externalSVGDocument && !m_errorOccurred && !m_loading && m_data) {
        m_externalSVGDocument = SVGDocument::create(0);
        m_externalSVGDocument->open();

        RefPtr<TextResourceDecoder> decoder = TextResourceDecoder::create("application/xml");
        m_externalSVGDocument->write(decoder->decode(m_data->data(), m_data->size()));
        m_externalSVGDocument->write(decoder->flush());
        if (decoder->sawError()) {
            m_externalSVGDocument.clear();
            return 0;
        }

        m_externalSVGDocument->finishParsing();
        m_externalSVGDocument->close();
    }

    return m_externalSVGDocument;
}

SVGFontElement* CachedFont::getSVGFontById(const String& fontName) const
{
    ASSERT(m_isSVGFont);
    RefPtr<NodeList> list = m_externalSVGDocument->getElementsByTagName(SVGNames::fontTag.localName());
    if (!list)
        return 0;

    unsigned fonts = list->length();
    for (unsigned i = 0; i < fonts; ++i) {
        Node* node = list->item(i);
        ASSERT(node);

        if (static_cast<Element*>(node)->getAttribute(static_cast<Element*>(node)->idAttributeName()) != fontName)
            continue;

        ASSERT(node->hasTagName(SVGNames::fontTag));
        return static_cast<SVGFontElement*>(node);
    }

    return 0;
}
#endif

void CachedFont::allClientsRemoved()
{
#ifdef STORE_FONT_CUSTOM_PLATFORM_DATA
    if (m_fontData) {
        delete m_fontData;
        m_fontData = 0;
    }
#endif
}

void CachedFont::checkNotify()
{
    if (m_loading)
        return;
    
    CachedResourceClientWalker w(m_clients);
    while (CachedResourceClient *c = w.next())
         c->fontLoaded(this);
}


void CachedFont::error()
{
    m_loading = false;
    m_errorOccurred = true;
    checkNotify();
}

}
