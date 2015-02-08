

#ifndef SVGFEComponentTransferElement_h
#define SVGFEComponentTransferElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterPrimitiveStandardAttributes.h"
#include "FEComponentTransfer.h"

namespace WebCore {

    class SVGFEComponentTransferElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEComponentTransferElement(const QualifiedName&, Document*);
        virtual ~SVGFEComponentTransferElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEComponentTransferElement, SVGNames::inAttr, String, In1, in1)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
