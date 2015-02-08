

#include "config.h"

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "SVGElement.h"
#include "SVGNames.h"

namespace WebCore {

SVGExternalResourcesRequired::SVGExternalResourcesRequired()
{
}

SVGExternalResourcesRequired::~SVGExternalResourcesRequired()
{
}

bool SVGExternalResourcesRequired::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::externalResourcesRequiredAttr) {
        setExternalResourcesRequiredBaseValue(attr->value() == "true");
        return true;
    }

    return false;
}

bool SVGExternalResourcesRequired::isKnownAttribute(const QualifiedName& attrName)
{
    return attrName == SVGNames::externalResourcesRequiredAttr;
}

}

#endif // ENABLE(SVG)
