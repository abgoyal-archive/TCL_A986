

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGGradientStop.h"

#include "SVGGradientElement.h"
#include "SVGNames.h"
#include "SVGStopElement.h"

namespace WebCore {
    
using namespace SVGNames;

RenderSVGGradientStop::RenderSVGGradientStop(SVGStopElement* element)
    : RenderObject(element)
{
}

RenderSVGGradientStop::~RenderSVGGradientStop()
{
}

void RenderSVGGradientStop::styleDidChange(StyleDifference diff, const RenderStyle* oldStyle)
{
    RenderObject::styleDidChange(diff, oldStyle);

    // <stop> elements should only be allowed to make renderers under gradient elements
    // but I can imagine a few cases we might not be catching, so let's not crash if our parent isn't a gradient.
    if (SVGGradientElement* gradient = gradientElement()) {
        if (SVGResource* resource = gradient->canvasResource(this))
            resource->invalidate();
    }
}

void RenderSVGGradientStop::layout()
{
    setNeedsLayout(false);
}

SVGGradientElement* RenderSVGGradientStop::gradientElement() const
{
    Node* parentNode = node()->parent();
    if (parentNode->hasTagName(linearGradientTag) || parentNode->hasTagName(radialGradientTag))
        return static_cast<SVGGradientElement*>(parentNode);
    return 0;
}

}

#endif // ENABLE(SVG)
