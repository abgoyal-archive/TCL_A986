

#ifndef SVGMissingGlyphElement_h
#define SVGMissingGlyphElement_h

#if ENABLE(SVG_FONTS)
#include "SVGStyledElement.h"

namespace WebCore {
    class SVGMissingGlyphElement : public SVGStyledElement {
    public:
        SVGMissingGlyphElement(const QualifiedName&, Document*);

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
