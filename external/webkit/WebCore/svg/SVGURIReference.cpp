

#include "config.h"

#if ENABLE(SVG)
#include "SVGURIReference.h"

#include "MappedAttribute.h"

namespace WebCore {

SVGURIReference::SVGURIReference()
{
}

SVGURIReference::~SVGURIReference()
{
}

bool SVGURIReference::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name().matches(XLinkNames::hrefAttr)) {
        setHrefBaseValue(attr->value());
        return true;
    }

    return false;
}

bool SVGURIReference::isKnownAttribute(const QualifiedName& attrName)
{
    return attrName.matches(XLinkNames::hrefAttr);
}

String SVGURIReference::getTarget(const String& url)
{
    if (url.startsWith("url(")) { // URI References, ie. fill:url(#target)
        unsigned int start = url.find('#') + 1;
        unsigned int end = url.reverseFind(')');

        return url.substring(start, end - start);
    } else if (url.find('#') > -1) { // format is #target
        unsigned int start = url.find('#') + 1;
        return url.substring(start, url.length() - start);
    } else // Normal Reference, ie. style="color-profile:changeColor"
        return url;
}

}

#endif // ENABLE(SVG)
