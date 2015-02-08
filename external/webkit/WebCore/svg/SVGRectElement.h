

#ifndef SVGRectElement_h
#define SVGRectElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGRectElement : public SVGStyledTransformableElement,
                           public SVGTests,
                           public SVGLangSpace,
                           public SVGExternalResourcesRequired {
    public:
        SVGRectElement(const QualifiedName&, Document*);
        virtual ~SVGRectElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual Path toPathData() const;

    protected:
        virtual bool hasRelativeValues() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::heightAttr, SVGLength, Height, height)
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::rxAttr, SVGLength, Rx, rx)
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::ryAttr, SVGLength, Ry, ry)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGRectElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
