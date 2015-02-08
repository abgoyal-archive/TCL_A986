

#ifndef SVGGradientElement_h
#define SVGGradientElement_h

#if ENABLE(SVG)
#include "RenderObject.h"
#include "SVGPaintServerGradient.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGStyledElement.h"
#include "SVGTransformList.h"
#include "SVGURIReference.h"

namespace WebCore {

    class SVGGradientElement : public SVGStyledElement,
                               public SVGURIReference,
                               public SVGExternalResourcesRequired {
    public:
        SVGGradientElement(const QualifiedName&, Document*);
        virtual ~SVGGradientElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

        virtual SVGResource* canvasResource(const RenderObject*);

    protected:
        friend class SVGPaintServerGradient;
        friend class SVGLinearGradientElement;
        friend class SVGRadialGradientElement;

        virtual void buildGradient() const = 0;
        virtual SVGPaintServerType gradientType() const = 0;

        Vector<SVGGradientStop> buildStops() const;
        mutable RefPtr<SVGPaintServerGradient> m_resource;
 
    protected:
        DECLARE_ANIMATED_PROPERTY(SVGGradientElement, SVGNames::spreadMethodAttr, int, SpreadMethod, spreadMethod)
        DECLARE_ANIMATED_PROPERTY(SVGGradientElement, SVGNames::gradientUnitsAttr, int, GradientUnits, gradientUnits)
        DECLARE_ANIMATED_PROPERTY(SVGGradientElement, SVGNames::gradientTransformAttr, SVGTransformList*, GradientTransform, gradientTransform)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGGradientElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGGradientElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
