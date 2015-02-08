

#include "config.h"
#if ENABLE(SVG)
#include "SVGMetadataElement.h"

using namespace WebCore;

SVGMetadataElement::SVGMetadataElement(const QualifiedName& tagName, Document *doc)
: SVGElement(tagName, doc)
{
}

SVGMetadataElement::~SVGMetadataElement()
{
}

#endif // ENABLE(SVG)
