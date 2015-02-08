

#ifndef SVGFECompositeElement_h
#define SVGFECompositeElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FEComposite.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    class SVGFECompositeElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFECompositeElement(const QualifiedName&, Document*);
        virtual ~SVGFECompositeElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::in2Attr, String, In2, in2)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::operatorAttr, int, _operator, _operator)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::k1Attr, float, K1, k1)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::k2Attr, float, K2, k2)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::k3Attr, float, K3, k3)
        DECLARE_ANIMATED_PROPERTY(SVGFECompositeElement, SVGNames::k4Attr, float, K4, k4)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
