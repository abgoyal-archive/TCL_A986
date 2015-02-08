

#include "config.h"

#if ENABLE(SVG)
#include "SVGTRefElement.h"

#include "RenderSVGInline.h"
#include "SVGDocument.h"
#include "SVGNames.h"
#include "Text.h"
#include "XLinkNames.h"

namespace WebCore {

SVGTRefElement::SVGTRefElement(const QualifiedName& tagName, Document* doc)
    : SVGTextPositioningElement(tagName, doc)
    , SVGURIReference()
{
}

SVGTRefElement::~SVGTRefElement()
{
}

void SVGTRefElement::updateReferencedText()
{
    Element* target = document()->getElementById(SVGURIReference::getTarget(href()));
    String textContent;
    if (target && target->isSVGElement())
        textContent = static_cast<SVGElement*>(target)->textContent();
    ExceptionCode ignore = 0;
    setTextContent(textContent, ignore);
}

void SVGTRefElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (SVGURIReference::parseMappedAttribute(attr)) {
        updateReferencedText();
        return;
    }

    SVGTextPositioningElement::parseMappedAttribute(attr);
}

void SVGTRefElement::svgAttributeChanged(const QualifiedName& attrName)
{
    SVGTextPositioningElement::svgAttributeChanged(attrName);

    if (!renderer())
        return;

    if (SVGURIReference::isKnownAttribute(attrName))
        renderer()->setNeedsLayout(true);
}

void SVGTRefElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGTextPositioningElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
}

bool SVGTRefElement::childShouldCreateRenderer(Node* child) const
{
    if (child->isTextNode() || child->hasTagName(SVGNames::tspanTag) ||
        child->hasTagName(SVGNames::trefTag))
        return true;
    return false;
}

RenderObject* SVGTRefElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGInline(this);
}

}

#endif // ENABLE(SVG)
