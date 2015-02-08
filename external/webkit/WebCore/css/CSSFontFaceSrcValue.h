

#ifndef CSSFontFaceSrcValue_h
#define CSSFontFaceSrcValue_h

#include "CSSValue.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>

#if ENABLE(SVG_FONTS)
#include "SVGFontFaceElement.h"
#endif

namespace WebCore {

class CSSFontFaceSrcValue : public CSSValue {
public:
    static PassRefPtr<CSSFontFaceSrcValue> create(const String& resource)
    {
        return adoptRef(new CSSFontFaceSrcValue(resource, false));
    }
    static PassRefPtr<CSSFontFaceSrcValue> createLocal(const String& resource)
    {
        return adoptRef(new CSSFontFaceSrcValue(resource, true));
    }

    virtual ~CSSFontFaceSrcValue() { }

    const String& resource() const { return m_resource; }
    const String& format() const { return m_format; }
    bool isLocal() const { return m_isLocal; }

    void setFormat(const String& format) { m_format = format; }

    bool isSupportedFormat() const;

#if ENABLE(SVG_FONTS)
    bool isSVGFontFaceSrc() const;

    SVGFontFaceElement* svgFontFaceElement() const { return m_svgFontFaceElement; }
    void setSVGFontFaceElement(SVGFontFaceElement* element) { m_svgFontFaceElement = element; }
#endif

    virtual String cssText() const;

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&, const CSSStyleSheet*);

private:
    CSSFontFaceSrcValue(const String& resource, bool local)
        : m_resource(resource)
        , m_isLocal(local)
#if ENABLE(SVG_FONTS)
        , m_svgFontFaceElement(0)
#endif
    {
    }

    String m_resource;
    String m_format;
    bool m_isLocal;

#if ENABLE(SVG_FONTS)
    SVGFontFaceElement* m_svgFontFaceElement;
#endif
};

}

#endif
