

#ifndef CSSFontFaceSource_h
#define CSSFontFaceSource_h

#include "AtomicString.h"
#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include <wtf/HashMap.h>

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceElement.h"
#endif

namespace WebCore {

class CachedFont;
class CSSFontFace;
class CSSFontSelector;
class FontDescription;
class SimpleFontData;

class CSSFontFaceSource : public CachedResourceClient {
public:
    CSSFontFaceSource(const String&, CachedFont* = 0);
    virtual ~CSSFontFaceSource();

    bool isLoaded() const;
    bool isValid() const;

    const AtomicString& string() const { return m_string; }

    void setFontFace(CSSFontFace* face) { m_face = face; }

    virtual void fontLoaded(CachedFont*);
    
    SimpleFontData* getFontData(const FontDescription&, bool syntheticBold, bool syntheticItalic, CSSFontSelector*);
    
    void pruneTable();

#if ENABLE(SVG_FONTS)
    SVGFontFaceElement* svgFontFaceElement() const { return m_svgFontFaceElement; }
    void setSVGFontFaceElement(SVGFontFaceElement* element) { m_svgFontFaceElement = element; }
#endif

private:
    AtomicString m_string; // URI for remote, built-in font name for local.
    CachedResourceHandle<CachedFont> m_font; // For remote fonts, a pointer to our cached resource.
    CSSFontFace* m_face; // Our owning font face.
    HashMap<unsigned, SimpleFontData*> m_fontDataTable; // The hash key is composed of size synthetic styles.

#if ENABLE(SVG_FONTS)
    SVGFontFaceElement* m_svgFontFaceElement;
    RefPtr<SVGFontElement> m_externalSVGFontElement;
#endif
};

}

#endif
