

#ifndef SVGFEMergeElement_h
#define SVGFEMergeElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEMerge.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    class SVGFEMergeElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEMergeElement(const QualifiedName&, Document*);
        virtual ~SVGFEMergeElement();

        virtual bool build(SVGResourceFilter*);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
