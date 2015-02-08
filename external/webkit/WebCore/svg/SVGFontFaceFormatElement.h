

#ifndef SVGFontFaceFormatElement_h
#define SVGFontFaceFormatElement_h

#if ENABLE(SVG_FONTS)
#include "SVGElement.h"

namespace WebCore {

class SVGFontFaceFormatElement : public SVGElement {
public:
    SVGFontFaceFormatElement(const QualifiedName&, Document*);

    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
};

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
