

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEFloodElement.h"

#include "MappedAttribute.h"
#include "RenderStyle.h"
#include "SVGRenderStyle.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEFloodElement::SVGFEFloodElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFEFloodElement::~SVGFEFloodElement()
{
}

bool SVGFEFloodElement::build(SVGResourceFilter* filterResource)
{
    RefPtr<RenderStyle> filterStyle = styleForRenderer();

    Color color = filterStyle->svgStyle()->floodColor();
    float opacity = filterStyle->svgStyle()->floodOpacity();

    RefPtr<FilterEffect> effect = FEFlood::create(color, opacity);
    filterResource->addFilterEffect(this, effect.release());
    
    return true;
}

}

#endif // ENABLE(SVG) && ENABLE(FILTERS)

// vim:ts=4:noet
