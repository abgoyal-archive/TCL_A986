

#ifndef SVGFontFaceUriElement_h
#define SVGFontFaceUriElement_h

#if ENABLE(SVG_FONTS)
#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "SVGElement.h"

namespace WebCore {

    class CSSFontFaceSrcValue;
    class CachedFont;

    class SVGFontFaceUriElement : public SVGElement, public CachedResourceClient {
    public:
        SVGFontFaceUriElement(const QualifiedName&, Document*);
        ~SVGFontFaceUriElement();
        
        PassRefPtr<CSSFontFaceSrcValue> srcValue() const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
        virtual void insertedIntoDocument();

    private:
        void loadFont();

        CachedResourceHandle<CachedFont> m_cachedFont;
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
