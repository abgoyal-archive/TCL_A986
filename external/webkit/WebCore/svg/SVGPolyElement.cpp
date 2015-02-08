

#include "config.h"

#if ENABLE(SVG)
#include "SVGPolyElement.h"

#include "Document.h"
#include "FloatPoint.h"
#include "MappedAttribute.h"
#include "RenderPath.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include "SVGPointList.h"

namespace WebCore {

SVGPolyElement::SVGPolyElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
    , SVGAnimatedPoints()
{
}

SVGPolyElement::~SVGPolyElement()
{
}

SVGPointList* SVGPolyElement::points() const
{
    if (!m_points)
        m_points = SVGPointList::create(SVGNames::pointsAttr);

    return m_points.get();
}

SVGPointList* SVGPolyElement::animatedPoints() const
{
    // FIXME!
    return 0;
}

void SVGPolyElement::parseMappedAttribute(MappedAttribute* attr)
{
    const AtomicString& value = attr->value();
    if (attr->name() == SVGNames::pointsAttr) {
        ExceptionCode ec = 0;
        points()->clear(ec);

        if (!pointsListFromSVGData(points(), value)) {
            points()->clear(ec);
            document()->accessSVGExtensions()->reportError("Problem parsing points=\"" + value + "\"");
        }
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

void SVGPolyElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::svgAttributeChanged(attrName);

    // The points property is not a regular SVGAnimatedProperty, still we use the same SVG<->XML DOM synchronization framework.
    if (attrName == SVGNames::pointsAttr)
        setSynchronizedSVGAttributes(false);

    if (!renderer())
        return;

    if (attrName == SVGNames::pointsAttr
        || SVGTests::isKnownAttribute(attrName)
        || SVGLangSpace::isKnownAttribute(attrName)
        || SVGExternalResourcesRequired::isKnownAttribute(attrName)
        || SVGStyledTransformableElement::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGPolyElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeExternalResourcesRequired();
        SVGAnimatedPropertySynchronizer<true>::synchronize(this, SVGNames::pointsAttr, points()->valueAsString());
        return;
    }

    if (SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
    else if (attrName == SVGNames::pointsAttr)
        SVGAnimatedPropertySynchronizer<true>::synchronize(this, attrName, points()->valueAsString());
}

}

#endif // ENABLE(SVG)
