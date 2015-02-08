

#ifndef SVGSetElement_h
#define SVGSetElement_h
#if ENABLE(SVG_ANIMATION)

#include "SVGAnimateElement.h"

namespace WebCore {
    // SVGAnimateElement implements superset of the functionality.
    class SVGSetElement : public SVGAnimateElement {
    public:
        SVGSetElement(const QualifiedName&, Document*);
    };

} // namespace WebCore

#endif // ENABLE(SVG_ANIMATION)
#endif

// vim:ts=4:noet
