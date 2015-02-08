

#ifndef SVGLinearGradientElement_h
#define SVGLinearGradientElement_h

#if ENABLE(SVG)
#include "SVGGradientElement.h"

namespace WebCore {

    struct LinearGradientAttributes;
    class SVGLength;

    class SVGLinearGradientElement : public SVGGradientElement {
    public:
        SVGLinearGradientElement(const QualifiedName&, Document*);
        virtual ~SVGLinearGradientElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

    protected:
        virtual void buildGradient() const;
        virtual SVGPaintServerType gradientType() const { return LinearGradientPaintServer; }

        LinearGradientAttributes collectGradientProperties() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGLinearGradientElement, SVGNames::x1Attr, SVGLength, X1, x1)
        DECLARE_ANIMATED_PROPERTY(SVGLinearGradientElement, SVGNames::y1Attr, SVGLength, Y1, y1)
        DECLARE_ANIMATED_PROPERTY(SVGLinearGradientElement, SVGNames::x2Attr, SVGLength, X2, x2)
        DECLARE_ANIMATED_PROPERTY(SVGLinearGradientElement, SVGNames::y2Attr, SVGLength, Y2, y2)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
