

#ifndef SVGTextPositioningElement_h
#define SVGTextPositioningElement_h

#if ENABLE(SVG)
#include "SVGTextContentElement.h"
#include "SVGLengthList.h"
#include "SVGNumberList.h"

namespace WebCore {

    class SVGTextPositioningElement : public SVGTextContentElement {
    public:
        SVGTextPositioningElement(const QualifiedName&, Document*);
        virtual ~SVGTextPositioningElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        bool isKnownAttribute(const QualifiedName&);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGTextPositioningElement, SVGNames::xAttr, SVGLengthList*, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGTextPositioningElement, SVGNames::yAttr, SVGLengthList*, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGTextPositioningElement, SVGNames::dxAttr, SVGLengthList*, Dx, dx)
        DECLARE_ANIMATED_PROPERTY(SVGTextPositioningElement, SVGNames::dyAttr, SVGLengthList*, Dy, dy)
        DECLARE_ANIMATED_PROPERTY(SVGTextPositioningElement, SVGNames::rotateAttr, SVGNumberList*, Rotate, rotate)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
