

#ifndef SVGEllipseElement_h
#define SVGEllipseElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGEllipseElement : public SVGStyledTransformableElement,
                              public SVGTests,
                              public SVGLangSpace,
                              public SVGExternalResourcesRequired {
    public:
        SVGEllipseElement(const QualifiedName&, Document*);
        virtual ~SVGEllipseElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual Path toPathData() const;

    protected:
        virtual bool hasRelativeValues() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGEllipseElement, SVGNames::cxAttr, SVGLength, Cx, cx)
        DECLARE_ANIMATED_PROPERTY(SVGEllipseElement, SVGNames::cyAttr, SVGLength, Cy, cy)
        DECLARE_ANIMATED_PROPERTY(SVGEllipseElement, SVGNames::rxAttr, SVGLength, Rx, rx)
        DECLARE_ANIMATED_PROPERTY(SVGEllipseElement, SVGNames::ryAttr, SVGLength, Ry, ry)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGEllipseElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
