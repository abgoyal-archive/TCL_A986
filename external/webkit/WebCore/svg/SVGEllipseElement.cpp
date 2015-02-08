

#include "config.h"

#if ENABLE(SVG)
#include "SVGEllipseElement.h"

#include "FloatPoint.h"
#include "MappedAttribute.h"
#include "RenderPath.h"
#include "SVGLength.h"
#include "SVGNames.h"

namespace WebCore {

SVGEllipseElement::SVGEllipseElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
    , m_cx(LengthModeWidth)
    , m_cy(LengthModeHeight)
    , m_rx(LengthModeWidth)
    , m_ry(LengthModeHeight)
{
}    

SVGEllipseElement::~SVGEllipseElement()
{
}

void SVGEllipseElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::cxAttr)
        setCxBaseValue(SVGLength(LengthModeWidth, attr->value()));
    else if (attr->name() == SVGNames::cyAttr)
        setCyBaseValue(SVGLength(LengthModeHeight, attr->value()));
    else if (attr->name() == SVGNames::rxAttr) {
        setRxBaseValue(SVGLength(LengthModeWidth, attr->value()));
        if (rxBaseValue().value(this) < 0.0)
            document()->accessSVGExtensions()->reportError("A negative value for ellipse <rx> is not allowed");
    } else if (attr->name() == SVGNames::ryAttr) {
        setRyBaseValue(SVGLength(LengthModeHeight, attr->value()));
        if (ryBaseValue().value(this) < 0.0)
            document()->accessSVGExtensions()->reportError("A negative value for ellipse <ry> is not allowed");
    } else {
        if (SVGTests::parseMappedAttribute(attr))
            return;
        if (SVGLangSpace::parseMappedAttribute(attr))
            return;
        if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
            return;
        SVGStyledTransformableElement::parseMappedAttribute(attr);
    }
}

void SVGEllipseElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::svgAttributeChanged(attrName);

    if (!renderer())
        return;

    if (attrName == SVGNames::cxAttr || attrName == SVGNames::cyAttr ||
        attrName == SVGNames::rxAttr || attrName == SVGNames::ryAttr ||
        SVGTests::isKnownAttribute(attrName) ||
        SVGLangSpace::isKnownAttribute(attrName) ||
        SVGExternalResourcesRequired::isKnownAttribute(attrName) ||
        SVGStyledTransformableElement::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGEllipseElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeCx();
        synchronizeCy();
        synchronizeRx();
        synchronizeRy();
        synchronizeExternalResourcesRequired();
        return;
    }

    if (attrName == SVGNames::cxAttr)
        synchronizeCx();
    else if (attrName == SVGNames::cyAttr)
        synchronizeCy();
    else if (attrName == SVGNames::rxAttr)
        synchronizeRx();
    else if (attrName == SVGNames::ryAttr)
        synchronizeRy();
    else if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

Path SVGEllipseElement::toPathData() const
{
    return Path::createEllipse(FloatPoint(cx().value(this), cy().value(this)),
                                          rx().value(this), ry().value(this));
}
 
bool SVGEllipseElement::hasRelativeValues() const
{
    return (cx().isRelative() || cy().isRelative() ||
            rx().isRelative() || ry().isRelative());
}

}

#endif // ENABLE(SVG)
