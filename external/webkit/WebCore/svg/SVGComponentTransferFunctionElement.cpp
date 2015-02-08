

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGComponentTransferFunctionElement.h"

#include "MappedAttribute.h"
#include "SVGFEComponentTransferElement.h"
#include "SVGNames.h"
#include "SVGNumberList.h"

namespace WebCore {

SVGComponentTransferFunctionElement::SVGComponentTransferFunctionElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
    , m_type(FECOMPONENTTRANSFER_TYPE_UNKNOWN)
    , m_tableValues(SVGNumberList::create(SVGNames::tableValuesAttr))
    , m_slope(1.0f)
    , m_amplitude(1.0f)
    , m_exponent(1.0f)
{
}

SVGComponentTransferFunctionElement::~SVGComponentTransferFunctionElement()
{
}

void SVGComponentTransferFunctionElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::typeAttr) {
        if (value == "identity")
            setTypeBaseValue(FECOMPONENTTRANSFER_TYPE_IDENTITY);
        else if (value == "table")
            setTypeBaseValue(FECOMPONENTTRANSFER_TYPE_TABLE);
        else if (value == "discrete")
            setTypeBaseValue(FECOMPONENTTRANSFER_TYPE_DISCRETE);
        else if (value == "linear")
            setTypeBaseValue(FECOMPONENTTRANSFER_TYPE_LINEAR);
        else if (value == "gamma")
            setTypeBaseValue(FECOMPONENTTRANSFER_TYPE_GAMMA);
    }
    else if (attr->name() == SVGNames::tableValuesAttr)
        tableValuesBaseValue()->parse(value);
    else if (attr->name() == SVGNames::slopeAttr)
        setSlopeBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::interceptAttr)
        setInterceptBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::amplitudeAttr)
        setAmplitudeBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::exponentAttr)
        setExponentBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::offsetAttr)
        setOffsetBaseValue(value.toFloat());
    else
        SVGElement::parseMappedAttribute(attr);
}

void SVGComponentTransferFunctionElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeType();
        synchronizeTableValues();
        synchronizeSlope();
        synchronizeIntercept();
        synchronizeAmplitude();
        synchronizeExponent();
        synchronizeOffset();
        return;
    }

    if (attrName == SVGNames::typeAttr)
        synchronizeType();
    else if (attrName == SVGNames::tableValuesAttr)
        synchronizeTableValues();
    else if (attrName == SVGNames::slopeAttr)
        synchronizeSlope();
    else if (attrName == SVGNames::interceptAttr)
        synchronizeIntercept();
    else if (attrName == SVGNames::amplitudeAttr)
        synchronizeAmplitude();
    else if (attrName == SVGNames::exponentAttr)
        synchronizeExponent();
    else if (attrName == SVGNames::offsetAttr)
        synchronizeOffset();
}

ComponentTransferFunction SVGComponentTransferFunctionElement::transferFunction() const
{
    ComponentTransferFunction func;
    func.type = (ComponentTransferType) type();
    func.slope = slope();
    func.intercept = intercept();
    func.amplitude = amplitude();
    func.exponent = exponent();
    func.offset = offset();
    SVGNumberList* numbers = tableValues();

    ExceptionCode ec = 0;
    unsigned int nr = numbers->numberOfItems();
    for (unsigned int i = 0; i < nr; i++)
        func.tableValues.append(numbers->getItem(i, ec));
    return func;
}

}

#endif // ENABLE(SVG)
