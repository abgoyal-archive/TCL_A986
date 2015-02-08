

#include "config.h"

#if ENABLE(SVG_FONTS)
#include "SVGAltGlyphElement.h"

#include "ExceptionCode.h"
#include "RenderInline.h"
#include "RenderSVGTSpan.h"
#include "SVGGlyphElement.h"
#include "SVGNames.h"
#include "XLinkNames.h"

namespace WebCore {

SVGAltGlyphElement::SVGAltGlyphElement(const QualifiedName& tagName, Document* doc)
    : SVGTextPositioningElement(tagName, doc)
{
}

SVGAltGlyphElement::~SVGAltGlyphElement()
{
}

void SVGAltGlyphElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGTextPositioningElement::synchronizeProperty(attrName);

    if (attrName == anyQName() || SVGURIReference::isKnownAttribute(attrName))
        synchronizeHref();
}

void SVGAltGlyphElement::setGlyphRef(const AtomicString&, ExceptionCode& ec)
{
    ec = NO_MODIFICATION_ALLOWED_ERR;
}

const AtomicString& SVGAltGlyphElement::glyphRef() const
{
    return getAttribute(SVGNames::glyphRefAttr);
}

void SVGAltGlyphElement::setFormat(const AtomicString&, ExceptionCode& ec)
{
    ec = NO_MODIFICATION_ALLOWED_ERR;
}

const AtomicString& SVGAltGlyphElement::format() const
{
    return getAttribute(SVGNames::formatAttr);
}

bool SVGAltGlyphElement::childShouldCreateRenderer(Node* child) const
{
    if (child->isTextNode())
        return true;
    return false;
}

RenderObject* SVGAltGlyphElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderSVGTSpan(this);
}

SVGGlyphElement* SVGAltGlyphElement::glyphElement() const
{
    Element* elt = document()->getElementById(getTarget(getAttribute(XLinkNames::hrefAttr)));
    if (!elt || !elt->hasTagName(SVGNames::glyphTag))
        return 0;
    return static_cast<SVGGlyphElement*>(elt);
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
