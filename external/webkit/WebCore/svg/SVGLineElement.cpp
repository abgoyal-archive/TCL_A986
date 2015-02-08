

#include "config.h"

#if ENABLE(SVG)
#include "SVGLineElement.h"

#include "FloatPoint.h"
#include "MappedAttribute.h"
#include "RenderPath.h"
#include "SVGLength.h"
#include "SVGNames.h"

namespace WebCore {

SVGLineElement::SVGLineElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
    , m_x1(LengthModeWidth)
    , m_y1(LengthModeHeight)
    , m_x2(LengthModeWidth)
    , m_y2(LengthModeHeight)
{
}

SVGLineElement::~SVGLineElement()
{
}

void SVGLineElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::x1Attr)
        setX1BaseValue(SVGLength(LengthModeWidth, attr->value()));
    else if (attr->name() == SVGNames::y1Attr)
        setY1BaseValue(SVGLength(LengthModeHeight, attr->value()));
    else if (attr->name() == SVGNames::x2Attr)
        setX2BaseValue(SVGLength(LengthModeWidth, attr->value()));
    else if (attr->name() == SVGNames::y2Attr)
        setY2BaseValue(SVGLength(LengthModeHeight, attr->value()));
    else {
        if (SVGTests::parseMappedAttribute(attr))
            return;
        if (SVGLangSpace::parseMappedAttribute(attr))
            return;
        if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
            return;
        SVGStyledTransformableElement::parseMappedAttribute(attr);
    }
}

void SVGLineElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::svgAttributeChanged(attrName);

    if (!renderer())
        return;

    if (attrName == SVGNames::x1Attr || attrName == SVGNames::y1Attr ||
        attrName == SVGNames::x2Attr || attrName == SVGNames::y2Attr ||
        SVGTests::isKnownAttribute(attrName) ||
        SVGLangSpace::isKnownAttribute(attrName) ||
        SVGExternalResourcesRequired::isKnownAttribute(attrName) ||
        SVGStyledTransformableElement::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGLineElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeX1();
        synchronizeY1();
        synchronizeX2();
        synchronizeY2();
        synchronizeExternalResourcesRequired();
        return;
    }

    if (attrName == SVGNames::x1Attr)
        synchronizeX1();
    else if (attrName == SVGNames::y1Attr)
        synchronizeY1();
    else if (attrName == SVGNames::x2Attr)
        synchronizeX2();
    else if (attrName == SVGNames::y2Attr)
        synchronizeY2();
    else if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

Path SVGLineElement::toPathData() const
{
    return Path::createLine(FloatPoint(x1().value(this), y1().value(this)),
                            FloatPoint(x2().value(this), y2().value(this)));
}

bool SVGLineElement::hasRelativeValues() const
{
    return (x1().isRelative() || y1().isRelative() ||
            x2().isRelative() || y2().isRelative());
}

}

#endif // ENABLE(SVG)
