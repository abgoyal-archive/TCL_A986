

#ifndef SVGFEFuncRElement_h
#define SVGFEFuncRElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGComponentTransferFunctionElement.h"

namespace WebCore {
    class SVGFEFuncRElement : public SVGComponentTransferFunctionElement {
    public:
        SVGFEFuncRElement(const QualifiedName&, Document*);
        virtual ~SVGFEFuncRElement();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
