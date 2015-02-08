

#ifndef SVGFETileElement_h
#define SVGFETileElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterPrimitiveStandardAttributes.h"
#include "SVGFETile.h"

namespace WebCore {

    class SVGFETileElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFETileElement(const QualifiedName&, Document*);
        virtual ~SVGFETileElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFETileElement, SVGNames::inAttr, String, In1, in1)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
