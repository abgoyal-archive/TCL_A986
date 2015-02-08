

#ifndef SVGFEFloodElement_h
#define SVGFEFloodElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFlood.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {
    class SVGFEFloodElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEFloodElement(const QualifiedName&, Document*);
        virtual ~SVGFEFloodElement();

        virtual bool build(SVGResourceFilter*);
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
#endif
