

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFELightElement.h"

#include "MappedAttribute.h"
#include "SVGNames.h"

namespace WebCore {

SVGFELightElement::SVGFELightElement(const QualifiedName& tagName, Document* doc)
    : SVGElement(tagName, doc)
    , m_specularExponent(1.0f)
{
}

SVGFELightElement::~SVGFELightElement()
{
}

void SVGFELightElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::azimuthAttr)
        setAzimuthBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::elevationAttr)
        setElevationBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::xAttr)
        setXBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::yAttr)
        setYBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::zAttr)
        setZBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::pointsAtXAttr)
        setPointsAtXBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::pointsAtYAttr)
        setPointsAtYBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::pointsAtZAttr)
        setPointsAtZBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::specularExponentAttr)
        setSpecularExponentBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::limitingConeAngleAttr)
        setLimitingConeAngleBaseValue(value.toFloat());
    else
        SVGElement::parseMappedAttribute(attr);
}

void SVGFELightElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeAzimuth();
        synchronizeElevation();
        synchronizeX();
        synchronizeY();
        synchronizeZ();
        synchronizePointsAtX();
        synchronizePointsAtY();
        synchronizePointsAtZ();
        synchronizeSpecularExponent();
        synchronizeLimitingConeAngle();
        return;
    }

    if (attrName == SVGNames::azimuthAttr)
        synchronizeAzimuth();
    else if (attrName == SVGNames::elevationAttr)
        synchronizeElevation();
    else if (attrName == SVGNames::xAttr)
        synchronizeX();
    else if (attrName == SVGNames::yAttr)
        synchronizeY();
    else if (attrName == SVGNames::zAttr)
        synchronizeZ();
    else if (attrName == SVGNames::pointsAtXAttr)
        synchronizePointsAtX();
    else if (attrName == SVGNames::pointsAtYAttr)
        synchronizePointsAtY();
    else if (attrName == SVGNames::pointsAtZAttr)
        synchronizePointsAtZ();
    else if (attrName == SVGNames::specularExponentAttr)
        synchronizeSpecularExponent();
    else if (attrName == SVGNames::limitingConeAngleAttr)
        synchronizeLimitingConeAngle();
}

}

#endif // ENABLE(SVG)
