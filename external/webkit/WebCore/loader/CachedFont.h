

#ifndef CachedFont_h
#define CachedFont_h

#include "CachedResource.h"
#include "FontRenderingMode.h"
#include <wtf/Vector.h>

#if ENABLE(SVG_FONTS)
#include "SVGElement.h"
#include "SVGDocument.h"
#endif

namespace WebCore {

class DocLoader;
class Cache;
class FontPlatformData;
class SVGFontElement;

struct FontCustomPlatformData;

class CachedFont : public CachedResource {
public:
    CachedFont(const String& url);
    virtual ~CachedFont();
    
    virtual void load(DocLoader* docLoader);

    virtual void didAddClient(CachedResourceClient*);
    virtual void data(PassRefPtr<SharedBuffer> data, bool allDataReceived);
    virtual void error();

    virtual void allClientsRemoved();

    virtual bool schedule() const { return true; }

    void checkNotify();

    void beginLoadIfNeeded(DocLoader* dl);

    bool ensureCustomFontData();
    FontPlatformData platformDataFromCustomData(float size, bool bold, bool italic, FontRenderingMode = NormalRenderingMode);

#if ENABLE(SVG_FONTS)
    bool isSVGFont() const { return m_isSVGFont; }
    void setSVGFont(bool isSVG) { m_isSVGFont = isSVG; }
    bool ensureSVGFontData();
    SVGFontElement* getSVGFontById(const String&) const;
#endif

private:
    FontCustomPlatformData* m_fontData;
    bool m_loadInitiated;

#if ENABLE(SVG_FONTS)
    bool m_isSVGFont;
    RefPtr<SVGDocument> m_externalSVGDocument;
#endif

    friend class Cache;
};

}

#endif
