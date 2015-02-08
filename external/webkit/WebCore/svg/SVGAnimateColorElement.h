

#ifndef SVGAnimateColorElement_h
#define SVGAnimateColorElement_h
#if ENABLE(SVG_ANIMATION)

#include "SVGAnimateElement.h"

namespace WebCore {

    // SVGAnimateElement implements superset of the functionality.
    class SVGAnimateColorElement : public SVGAnimateElement {
    public:
        SVGAnimateColorElement(const QualifiedName&, Document*);
    };

} // namespace WebCore

#endif // ENABLE(SVG_ANIMATION)
#endif // KSVG_SVGAnimateColorElementImpl_H

// vim:ts=4:noet
