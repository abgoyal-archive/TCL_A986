

#include "config.h"

#if ENABLE(SVG)
#include "SVGMPathElement.h"

#include "Document.h"
#include "SVGPathElement.h"

namespace WebCore {

SVGMPathElement::SVGMPathElement(const QualifiedName& qname, Document* doc)
    : SVGElement(qname, doc)
{
}

SVGMPathElement::~SVGMPathElement()
{
}

void SVGMPathElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (SVGURIReference::parseMappedAttribute(attr))
        return;
    SVGElement::parseMappedAttribute(attr);
}

void SVGMPathElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeExternalResourcesRequired();
        synchronizeHref();
        return;
    }

    if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
    else if (SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
}

SVGPathElement* SVGMPathElement::pathElement()
{
    Element* target = document()->getElementById(getTarget(href()));
    if (target && target->hasTagName(SVGNames::pathTag))
        return static_cast<SVGPathElement*>(target);
    return 0;
}

} // namespace WebCore

#endif // ENABLE(SVG)
