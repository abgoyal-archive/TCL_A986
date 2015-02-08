

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGHiddenContainer.h"

#include "RenderPath.h"
#include "SVGStyledElement.h"

namespace WebCore {

RenderSVGHiddenContainer::RenderSVGHiddenContainer(SVGStyledElement* element)
    : RenderSVGContainer(element)
{
}

void RenderSVGHiddenContainer::layout()
{
    ASSERT(needsLayout());
    layoutChildren(this, selfNeedsLayout()); 
    setNeedsLayout(false);    
}

void RenderSVGHiddenContainer::paint(PaintInfo&, int, int)
{
    // This subtree does not paint.
}

IntRect RenderSVGHiddenContainer::clippedOverflowRectForRepaint(RenderBoxModelObject* /*repaintContainer*/)
{
    return IntRect();
}

void RenderSVGHiddenContainer::absoluteRects(Vector<IntRect>&, int, int)
{
    // This subtree does not take up space or paint
}

void RenderSVGHiddenContainer::absoluteQuads(Vector<FloatQuad>&)
{
    // This subtree does not take up space or paint
}

bool RenderSVGHiddenContainer::nodeAtFloatPoint(const HitTestRequest&, HitTestResult&, const FloatPoint&, HitTestAction)
{
    return false;
}

FloatRect RenderSVGHiddenContainer::objectBoundingBox() const
{
    return FloatRect();
}

FloatRect RenderSVGHiddenContainer::repaintRectInLocalCoordinates() const
{
    return FloatRect();
}

}

#endif // ENABLE(SVG)
