

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterPrimitiveStandardAttributes.h"

#include "FilterEffect.h"
#include "MappedAttribute.h"
#include "SVGLength.h"
#include "SVGNames.h"
#include "SVGStyledElement.h"
#include "SVGUnitTypes.h"

namespace WebCore {

SVGFilterPrimitiveStandardAttributes::SVGFilterPrimitiveStandardAttributes(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
    , m_x(LengthModeWidth, "0%")
    , m_y(LengthModeHeight, "0%")
    , m_width(LengthModeWidth, "100%")
    , m_height(LengthModeHeight, "100%")
{
    // Spec: If the x/y attribute is not specified, the effect is as if a value of "0%" were specified.
    // Spec: If the width/height attribute is not specified, the effect is as if a value of "100%" were specified.
}

SVGFilterPrimitiveStandardAttributes::~SVGFilterPrimitiveStandardAttributes()
{
}

void SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(MappedAttribute* attr)
{
    const AtomicString& value = attr->value();
    if (attr->name() == SVGNames::xAttr)
        setXBaseValue(SVGLength(LengthModeWidth, value));
    else if (attr->name() == SVGNames::yAttr)
        setYBaseValue(SVGLength(LengthModeHeight, value));
    else if (attr->name() == SVGNames::widthAttr)
        setWidthBaseValue(SVGLength(LengthModeWidth, value));
    else if (attr->name() == SVGNames::heightAttr)
        setHeightBaseValue(SVGLength(LengthModeHeight, value));
    else if (attr->name() == SVGNames::resultAttr)
        setResultBaseValue(value);
    else
        return SVGStyledElement::parseMappedAttribute(attr);
}

void SVGFilterPrimitiveStandardAttributes::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeX();
        synchronizeY();
        synchronizeWidth();
        synchronizeHeight();
        synchronizeResult();
        return;
    }

    if (attrName == SVGNames::xAttr)
        synchronizeX();
    else if (attrName == SVGNames::yAttr)
        synchronizeY();
    else if (attrName == SVGNames::widthAttr)
        synchronizeWidth();
    else if (attrName == SVGNames::heightAttr)
        synchronizeHeight();
    else if (attrName == SVGNames::resultAttr)
        synchronizeResult();
}

void SVGFilterPrimitiveStandardAttributes::setStandardAttributes(SVGResourceFilter* resourceFilter, FilterEffect* filterEffect) const
{
    ASSERT(filterEffect);
    if (!filterEffect)
        return;

    ASSERT(resourceFilter);

    if (this->hasAttribute(SVGNames::xAttr))
        filterEffect->setHasX(true);
    if (this->hasAttribute(SVGNames::yAttr))
        filterEffect->setHasY(true);
    if (this->hasAttribute(SVGNames::widthAttr))
        filterEffect->setHasWidth(true);
    if (this->hasAttribute(SVGNames::heightAttr))
        filterEffect->setHasHeight(true);

    FloatRect effectBBox;
    if (resourceFilter->effectBoundingBoxMode())
        effectBBox = FloatRect(x().valueAsPercentage(),
                               y().valueAsPercentage(),
                               width().valueAsPercentage(),
                               height().valueAsPercentage());
    else
        effectBBox = FloatRect(x().value(this),
                               y().value(this),
                               width().value(this),
                               height().value(this));

    filterEffect->setEffectBoundaries(effectBBox);
}

}

#endif // ENABLE(SVG)
