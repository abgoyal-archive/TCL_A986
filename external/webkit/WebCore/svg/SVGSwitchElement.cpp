

#include "config.h"
#if ENABLE(SVG)
#include "SVGSwitchElement.h"

#include "RenderSVGTransformableContainer.h"
#include "SVGNames.h"

namespace WebCore {

SVGSwitchElement::SVGSwitchElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledTransformableElement(tagName, doc)
    , SVGTests()
    , SVGLangSpace()
    , SVGExternalResourcesRequired()
{
}

SVGSwitchElement::~SVGSwitchElement()
{
}

bool SVGSwitchElement::childShouldCreateRenderer(Node* child) const
{
    for (Node* n = firstChild(); n != 0; n = n->nextSibling()) {
        if (n->isSVGElement()) {
            SVGElement* element = static_cast<SVGElement*>(n);
            if (element && element->isValid())
                return (n == child); // Only allow this child if it's the first valid child
        }
    }

    return false;
}

RenderObject* SVGSwitchElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGTransformableContainer(this);
}

void SVGSwitchElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGStyledTransformableElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGExternalResourcesRequired::isKnownAttribute(attrName))
        synchronizeExternalResourcesRequired();
}

}

#endif // ENABLE(SVG)
