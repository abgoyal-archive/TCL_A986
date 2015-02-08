

#ifndef SVGCircleElement_h
#define SVGCircleElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGCircleElement : public SVGStyledTransformableElement,
                             public SVGTests,
                             public SVGLangSpace,
                             public SVGExternalResourcesRequired {
    public:
        SVGCircleElement(const QualifiedName&, Document*);
        virtual ~SVGCircleElement();

        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual Path toPathData() const;

    protected:
        virtual bool hasRelativeValues() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGCircleElement, SVGNames::cxAttr, SVGLength, Cx, cx)
        DECLARE_ANIMATED_PROPERTY(SVGCircleElement, SVGNames::cyAttr, SVGLength, Cy, cy)
        DECLARE_ANIMATED_PROPERTY(SVGCircleElement, SVGNames::rAttr, SVGLength, R, r)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGCircleElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGCircleElement_h
