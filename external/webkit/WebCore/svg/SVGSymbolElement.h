

#ifndef SVGSymbolElement_h
#define SVGSymbolElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGFitToViewBox.h"
#include "SVGLangSpace.h"
#include "SVGStyledElement.h"

namespace WebCore {

    class SVGSymbolElement : public SVGStyledElement,
                             public SVGLangSpace,
                             public SVGExternalResourcesRequired,
                             public SVGFitToViewBox {
    public:
        SVGSymbolElement(const QualifiedName&, Document*);
        virtual ~SVGSymbolElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

    private:
        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGSymbolElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
 
        // SVGFitToViewBox
        DECLARE_ANIMATED_PROPERTY(SVGSymbolElement, SVGNames::viewBoxAttr, FloatRect, ViewBox, viewBox)
        DECLARE_ANIMATED_PROPERTY(SVGSymbolElement, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio) 
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
