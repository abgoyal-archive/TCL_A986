

#ifndef SVGRadialGradientElement_h
#define SVGRadialGradientElement_h

#if ENABLE(SVG)
#include "SVGGradientElement.h"

namespace WebCore {

    struct RadialGradientAttributes;
    class SVGLength;

    class SVGRadialGradientElement : public SVGGradientElement {
    public:
        SVGRadialGradientElement(const QualifiedName&, Document*);
        virtual ~SVGRadialGradientElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

    protected:
        virtual void buildGradient() const;
        virtual SVGPaintServerType gradientType() const { return RadialGradientPaintServer; }

        RadialGradientAttributes collectGradientProperties() const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGRadialGradientElement, SVGNames::cxAttr, SVGLength, Cx, cx)
        DECLARE_ANIMATED_PROPERTY(SVGRadialGradientElement, SVGNames::cyAttr, SVGLength, Cy, cy)
        DECLARE_ANIMATED_PROPERTY(SVGRadialGradientElement, SVGNames::rAttr, SVGLength, R, r)
        DECLARE_ANIMATED_PROPERTY(SVGRadialGradientElement, SVGNames::fxAttr, SVGLength, Fx, fx)
        DECLARE_ANIMATED_PROPERTY(SVGRadialGradientElement, SVGNames::fyAttr, SVGLength, Fy, fy)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
