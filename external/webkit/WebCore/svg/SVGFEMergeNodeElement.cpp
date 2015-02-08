

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEMergeNodeElement.h"

#include "MappedAttribute.h"

namespace WebCore {

SVGFEMergeNodeElement::SVGFEMergeNodeElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
{
}

SVGFEMergeNodeElement::~SVGFEMergeNodeElement()
{
}

void SVGFEMergeNodeElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else
        SVGElement::parseMappedAttribute(attr);
}

void SVGFEMergeNodeElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || attrName == SVGNames::inAttr)
        synchronizeIn1();
}

}

#endif // ENABLE(SVG)
