

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFuncAElement.h"

namespace WebCore {

SVGFEFuncAElement::SVGFEFuncAElement(const QualifiedName& tagName, Document* doc)
    : SVGComponentTransferFunctionElement(tagName, doc)
{
}

SVGFEFuncAElement::~SVGFEFuncAElement()
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
