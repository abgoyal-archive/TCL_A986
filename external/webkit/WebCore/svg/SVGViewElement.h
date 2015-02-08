

#ifndef SVGViewElement_h
#define SVGViewElement_h

#if ENABLE(SVG)
#include "SVGStyledElement.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGFitToViewBox.h"
#include "SVGZoomAndPan.h"

namespace WebCore {

    class SVGStringList;
    class SVGViewElement : public SVGStyledElement,
                           public SVGExternalResourcesRequired,
                           public SVGFitToViewBox,
                           public SVGZoomAndPan {
    public:
        SVGViewElement(const QualifiedName&, Document*);
        virtual ~SVGViewElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);

        SVGStringList* viewTarget() const;

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

    private:
        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGViewElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
 
        // SVGFitToViewBox
        DECLARE_ANIMATED_PROPERTY(SVGViewElement, SVGNames::viewBoxAttr, FloatRect, ViewBox, viewBox)
        DECLARE_ANIMATED_PROPERTY(SVGViewElement, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio)
 
        mutable RefPtr<SVGStringList> m_viewTarget;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
