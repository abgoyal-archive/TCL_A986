

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFuncRElement.h"

namespace WebCore {

SVGFEFuncRElement::SVGFEFuncRElement(const QualifiedName& tagName, Document* doc)
    : SVGComponentTransferFunctionElement(tagName, doc)
{
}

SVGFEFuncRElement::~SVGFEFuncRElement()
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
