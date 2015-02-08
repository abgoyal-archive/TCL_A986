

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEGaussianBlurElement.h"

#include "MappedAttribute.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include "SVGResourceFilter.h"

namespace WebCore {

char SVGStdDeviationXAttrIdentifier[] = "SVGStdDeviationXAttr";
char SVGStdDeviationYAttrIdentifier[] = "SVGStdDeviationYAttr";

SVGFEGaussianBlurElement::SVGFEGaussianBlurElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFEGaussianBlurElement::~SVGFEGaussianBlurElement()
{
}

void SVGFEGaussianBlurElement::setStdDeviation(float, float)
{
    // FIXME: Needs an implementation.
}

void SVGFEGaussianBlurElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::stdDeviationAttr) {
        float x, y;
        if (parseNumberOptionalNumber(value, x, y)) {
            setStdDeviationXBaseValue(x);
            setStdDeviationYBaseValue(y);
        }
    } else if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFEGaussianBlurElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeStdDeviationX();
        synchronizeStdDeviationY();
        synchronizeIn1();
        return;
    }

    if (attrName == SVGNames::stdDeviationAttr) {
        synchronizeStdDeviationX();
        synchronizeStdDeviationY();
    } else if (attrName == SVGNames::inAttr)
        synchronizeIn1();
}

bool SVGFEGaussianBlurElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());

    if (!input1)
        return false;

    RefPtr<FilterEffect> effect = FEGaussianBlur::create(input1, stdDeviationX(), stdDeviationY());
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)
