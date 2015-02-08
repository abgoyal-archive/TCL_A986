

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFuncBElement.h"

namespace WebCore {

SVGFEFuncBElement::SVGFEFuncBElement(const QualifiedName& tagName, Document *doc)
    : SVGComponentTransferFunctionElement(tagName, doc)
{
}

SVGFEFuncBElement::~SVGFEFuncBElement()
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
