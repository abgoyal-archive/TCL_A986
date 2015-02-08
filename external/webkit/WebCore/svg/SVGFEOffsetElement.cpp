

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEOffsetElement.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEOffsetElement::SVGFEOffsetElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFEOffsetElement::~SVGFEOffsetElement()
{
}

void SVGFEOffsetElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::dxAttr)
        setDxBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::dyAttr)
        setDyBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFEOffsetElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeDx();
        synchronizeDy();
        synchronizeIn1();
        return;
    }

    if (attrName == SVGNames::dxAttr)
        synchronizeDx();
    else if (attrName == SVGNames::dyAttr)
        synchronizeDy();
    else if (attrName == SVGNames::inAttr)
        synchronizeIn1();
}

bool SVGFEOffsetElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());

    if (!input1)
        return false;

    RefPtr<FilterEffect> effect = FEOffset::create(input1, dx(), dy());
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)
