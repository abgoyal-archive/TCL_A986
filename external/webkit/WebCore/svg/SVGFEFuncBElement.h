

#ifndef SVGFEFuncBElement_h
#define SVGFEFuncBElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGComponentTransferFunctionElement.h"

namespace WebCore {
    class SVGFEFuncBElement : public SVGComponentTransferFunctionElement {
    public:
        SVGFEFuncBElement(const QualifiedName&, Document*);
        virtual ~SVGFEFuncBElement();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
