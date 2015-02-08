

#ifndef SVGFontFaceNameElement_h
#define SVGFontFaceNameElement_h

#if ENABLE(SVG_FONTS)
#include "SVGElement.h"

namespace WebCore {
    class CSSFontFaceSrcValue;
    class SVGFontFaceNameElement : public SVGElement {
    public:
        SVGFontFaceNameElement(const QualifiedName&, Document*);
        
        PassRefPtr<CSSFontFaceSrcValue> srcValue() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
