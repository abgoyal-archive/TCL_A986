

#include "config.h"

#if ENABLE(SVG)
#include "SVGDefsElement.h"

#include "RenderSVGHiddenContainer.h"

namespace WebCore {

SVGDefsElement::SVGDefsElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
{
}

SVGDefsElement::~SVGDefsElement()
{
}

bool SVGDefsElement::isValid() const
{
    return SVGTests::isValid();
}

RenderObject* SVGDefsElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGHiddenContainer(this);
}

void SVGDefsElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

}

#endif // ENABLE(SVG)
