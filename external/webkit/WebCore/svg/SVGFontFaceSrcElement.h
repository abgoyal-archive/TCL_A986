

#ifndef SVGFontFaceSrcElement_h
#define SVGFontFaceSrcElement_h

#if ENABLE(SVG_FONTS)
#include "SVGElement.h"

namespace WebCore {
    class CSSValueList;
    class SVGFontFaceSrcElement : public SVGElement {
    public:
        SVGFontFaceSrcElement(const QualifiedName&, Document*);
        
        PassRefPtr<CSSValueList> srcValue() const;
        
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
