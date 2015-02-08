

#ifndef SVGHKernElement_h
#define SVGHKernElement_h

#if ENABLE(SVG_FONTS)
#include "SVGStyledElement.h"

#include <limits>
#include "Path.h"

namespace WebCore {

    class AtomicString;
    class SVGFontData;

    // Describe an SVG <hkern> element
    struct SVGHorizontalKerningPair {
        String unicode1;
        String glyphName1;
        String unicode2;
        String glyphName2;
        double kerning;
        
        SVGHorizontalKerningPair()
            : kerning(0)
        {
        }
    };

    class SVGHKernElement : public SVGElement {
    public:
        SVGHKernElement(const QualifiedName&, Document*);
        virtual ~SVGHKernElement();

        virtual void insertedIntoDocument();
        virtual void removedFromDocument();

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

        SVGHorizontalKerningPair buildHorizontalKerningPair() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif
