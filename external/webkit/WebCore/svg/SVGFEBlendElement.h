

#ifndef SVGFEBlendElement_h
#define SVGFEBlendElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FEBlend.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {
    class SVGFEBlendElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEBlendElement(const QualifiedName&, Document*);
        virtual ~SVGFEBlendElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEBlendElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEBlendElement, SVGNames::in2Attr, String, In2, in2)
        DECLARE_ANIMATED_PROPERTY(SVGFEBlendElement, SVGNames::modeAttr, int, Mode, mode)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
