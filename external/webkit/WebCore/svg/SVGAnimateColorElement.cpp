

#include "config.h"
#if ENABLE(SVG_ANIMATION)
#include "SVGAnimateColorElement.h"

namespace WebCore {
    
SVGAnimateColorElement::SVGAnimateColorElement(const QualifiedName& tagName, Document* doc)
    : SVGAnimateElement(tagName, doc)
{
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)

