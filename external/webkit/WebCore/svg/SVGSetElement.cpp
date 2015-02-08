

#include "config.h"
#if ENABLE(SVG_ANIMATION)
#include "SVGSetElement.h"

namespace WebCore {
    
SVGSetElement::SVGSetElement(const QualifiedName& tagName, Document *doc)
    : SVGAnimateElement(tagName, doc)
{
}
}

// vim:ts=4:noet
#endif // ENABLE(SVG_ANIMATION)

