

#ifndef SVGFEFuncAElement_h
#define SVGFEFuncAElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGComponentTransferFunctionElement.h"

namespace WebCore {
    class SVGFEFuncAElement : public SVGComponentTransferFunctionElement {
    public:
        SVGFEFuncAElement(const QualifiedName&, Document*);
        virtual ~SVGFEFuncAElement();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
