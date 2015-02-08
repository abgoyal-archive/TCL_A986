

#ifndef SVGFEOffsetElement_h
#define SVGFEOffsetElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterPrimitiveStandardAttributes.h"
#include "SVGFEOffset.h"

namespace WebCore {

    class SVGFEOffsetElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEOffsetElement(const QualifiedName&, Document*);
        virtual ~SVGFEOffsetElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEOffsetElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEOffsetElement, SVGNames::dxAttr, float, Dx, dx)
        DECLARE_ANIMATED_PROPERTY(SVGFEOffsetElement, SVGNames::dyAttr, float, Dy, dy)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
