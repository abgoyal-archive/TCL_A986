

#ifndef SVGLineElement_h
#define SVGLineElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGLength;

    class SVGLineElement : public SVGStyledTransformableElement,
                           public SVGTests,
                           public SVGLangSpace,
                           public SVGExternalResourcesRequired {
    public:
        SVGLineElement(const QualifiedName&, Document*);
        virtual ~SVGLineElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual Path toPathData() const;

        virtual bool supportsMarkers() const { return true; }

    protected:
        virtual bool hasRelativeValues() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGLineElement, SVGNames::x1Attr, SVGLength, X1, x1)
        DECLARE_ANIMATED_PROPERTY(SVGLineElement, SVGNames::y1Attr, SVGLength, Y1, y1)
        DECLARE_ANIMATED_PROPERTY(SVGLineElement, SVGNames::x2Attr, SVGLength, X2, x2)
        DECLARE_ANIMATED_PROPERTY(SVGLineElement, SVGNames::y2Attr, SVGLength, Y2, y2)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGLineElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
