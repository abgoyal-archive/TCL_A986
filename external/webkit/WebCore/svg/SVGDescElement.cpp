

#include "config.h"
#if ENABLE(SVG)
#include "SVGDescElement.h"

namespace WebCore {

SVGDescElement::SVGDescElement(const QualifiedName& tagName, Document *doc)
    : SVGStyledElement(tagName, doc)
    , SVGLangSpace()
{
}

SVGDescElement::~SVGDescElement()
{
}

String SVGDescElement::description() const
{
    return textContent().simplifyWhiteSpace();
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)

