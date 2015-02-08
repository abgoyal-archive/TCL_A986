

#ifndef SVGFEFuncGElement_h
#define SVGFEFuncGElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGComponentTransferFunctionElement.h"

namespace WebCore {
    class SVGFEFuncGElement : public SVGComponentTransferFunctionElement {
    public:
        SVGFEFuncGElement(const QualifiedName&, Document*);
        virtual ~SVGFEFuncGElement();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
