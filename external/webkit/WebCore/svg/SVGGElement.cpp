

#include "config.h"

#if ENABLE(SVG)
#include "SVGGElement.h"

#include "RenderSVGTransformableContainer.h"

namespace WebCore {

SVGGElement::SVGGElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
{
}

SVGGElement::~SVGGElement()
{
}

void SVGGElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (SVGTests::parseMappedAttribute(attr))
        return;
    if (SVGLangSpace::parseMappedAttribute(attr))
        return;
    if (SVGExternalResourcesRequired::parseMappedAttribute(attr))
        return;

    SVGStyledTransformableElement::parseMappedAttribute(attr);
}

void SVGGElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::svgAttributeChanged(attrName);

    if (!renderer())
        return;

    if (SVGTests::isKnownAttribute(attrName) || 
        SVGLangSpace::isKnownAttribute(attrName) ||
        SVGExternalResourcesRequired::isKnownAttribute(attrName) ||
        SVGStyledTransformableElement::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGGElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

void SVGGElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGStyledTransformableElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);

    if (renderer())
        renderer()->setNeedsLayout(true);
}

RenderObject* SVGGElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGTransformableContainer(this);
}

}

#endif // ENABLE(SVG)
