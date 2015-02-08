

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFETileElement.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "SVGRenderStyle.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFETileElement::SVGFETileElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFETileElement::~SVGFETileElement()
{
}

void SVGFETileElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFETileElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName() || attrName == SVGNames::inAttr)
        synchronizeIn1();
}

bool SVGFETileElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());

    if (!input1)
        return false;

    RefPtr<FilterEffect> effect = FETile::create(input1);
    filterResource->addFilterEffect(this, effect.release());
    
    return true;
}

}

#endif // ENABLE(SVG)
