

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEBlendElement.h"

#include "MappedAttribute.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEBlendElement::SVGFEBlendElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
    , m_mode(FEBLEND_MODE_NORMAL)
{
}

SVGFEBlendElement::~SVGFEBlendElement()
{
}

void SVGFEBlendElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::modeAttr) {
        if (value == "normal")
            setModeBaseValue(FEBLEND_MODE_NORMAL);
        else if (value == "multiply")
            setModeBaseValue(FEBLEND_MODE_MULTIPLY);
        else if (value == "screen")
            setModeBaseValue(FEBLEND_MODE_SCREEN);
        else if (value == "darken")
            setModeBaseValue(FEBLEND_MODE_DARKEN);
        else if (value == "lighten")
            setModeBaseValue(FEBLEND_MODE_LIGHTEN);
    } else if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else if (attr->name() == SVGNames::in2Attr)
        setIn2BaseValue(value);
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFEBlendElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeMode();
        synchronizeIn1();
        synchronizeIn2();
        return;
    }

    if (attrName == SVGNames::modeAttr)
        synchronizeMode();
    else if (attrName == SVGNames::inAttr)
        synchronizeIn1();
    else if (attrName == SVGNames::in2Attr)
        synchronizeIn2();
}

bool SVGFEBlendElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());
    FilterEffect* input2 = filterResource->builder()->getEffectById(in2());

    if (!input1 || !input2)
        return false;

    RefPtr<FilterEffect> effect = FEBlend::create(input1, input2, static_cast<BlendModeType>(mode()));
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
