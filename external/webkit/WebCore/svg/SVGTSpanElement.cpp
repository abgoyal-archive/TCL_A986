

#include "config.h"

#if ENABLE(SVG)
#include "SVGTSpanElement.h"

#include "RenderInline.h"
#include "RenderSVGTSpan.h"
#include "SVGNames.h"

namespace WebCore {

SVGTSpanElement::SVGTSpanElement(const QualifiedName& tagName, Document* doc)
    : SVGTextPositioningElement(tagName, doc)
{
}

SVGTSpanElement::~SVGTSpanElement()
{
}

bool SVGTSpanElement::childShouldCreateRenderer(Node* child) const
{
    if (child->isTextNode()
#if ENABLE(SVG_FONTS)
        || child->hasTagName(SVGNames::altGlyphTag)
#endif
        || child->hasTagName(SVGNames::tspanTag) || child->hasTagName(SVGNames::trefTag) || child->hasTagName(SVGNames::textPathTag))
        return true;

    return false;
}

RenderObject* SVGTSpanElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGTSpan(this);
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
