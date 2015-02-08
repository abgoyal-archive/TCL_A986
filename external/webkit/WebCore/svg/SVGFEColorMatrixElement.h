

#ifndef SVGFEColorMatrixElement_h
#define SVGFEColorMatrixElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FEColorMatrix.h"
#include "SVGFilterPrimitiveStandardAttributes.h"
#include "SVGNumberList.h"

namespace WebCore {

    class SVGFEColorMatrixElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEColorMatrixElement(const QualifiedName&, Document*);
        virtual ~SVGFEColorMatrixElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEColorMatrixElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEColorMatrixElement, SVGNames::typeAttr, int, Type, type)
        DECLARE_ANIMATED_PROPERTY(SVGFEColorMatrixElement, SVGNames::valuesAttr, SVGNumberList*, Values, values)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
