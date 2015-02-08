

#include "config.h"

#if ENABLE(SVG)
#include "SVGStyledLocatableElement.h"

#include "AffineTransform.h"
#include "RenderPath.h"
#include "SVGElement.h"
#include "SVGSVGElement.h"

namespace WebCore {

SVGStyledLocatableElement::SVGStyledLocatableElement(const QualifiedName& tagName, Document* doc)
    : SVGLocatable()
    , SVGStyledElement(tagName, doc)
{
}

SVGStyledLocatableElement::~SVGStyledLocatableElement()
{
}

SVGElement* SVGStyledLocatableElement::nearestViewportElement() const
{
    return SVGLocatable::nearestViewportElement(this);
}

SVGElement* SVGStyledLocatableElement::farthestViewportElement() const
{
    return SVGLocatable::farthestViewportElement(this);
}

FloatRect SVGStyledLocatableElement::getBBox() const
{
    return SVGLocatable::getBBox(this);
}

AffineTransform SVGStyledLocatableElement::getCTM() const
{
    return SVGLocatable::getCTM(this);
}

AffineTransform SVGStyledLocatableElement::getScreenCTM() const
{
    return SVGLocatable::getScreenCTM(this);
}

}

#endif // ENABLE(SVG)
