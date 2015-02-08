

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFuncGElement.h"

namespace WebCore {

SVGFEFuncGElement::SVGFEFuncGElement(const QualifiedName& tagName, Document* doc)
    : SVGComponentTransferFunctionElement(tagName, doc)
{
}

SVGFEFuncGElement::~SVGFEFuncGElement()
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
