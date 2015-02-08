

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGModelObject.h"

#include "GraphicsContext.h"
#include "RenderLayer.h"
#include "RenderView.h"
#include "SVGStyledElement.h"

#if ENABLE(FILTERS)
#include "SVGResourceFilter.h"
#endif

namespace WebCore {

RenderSVGModelObject::RenderSVGModelObject(SVGStyledElement* node)
    : RenderObject(node)
{
}

IntRect RenderSVGModelObject::clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer)
{
    return SVGRenderBase::clippedOverflowRectForRepaint(this, repaintContainer);
}

void RenderSVGModelObject::computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect& repaintRect, bool fixed)
{
    SVGRenderBase::computeRectForRepaint(this, repaintContainer, repaintRect, fixed);
}

void RenderSVGModelObject::mapLocalToContainer(RenderBoxModelObject* repaintContainer, bool fixed , bool useTransforms, TransformState& transformState) const
{
    SVGRenderBase::mapLocalToContainer(this, repaintContainer, fixed, useTransforms, transformState);
}

// Copied from RenderBox, this method likely requires further refactoring to work easily for both SVG and CSS Box Model content.
// FIXME: This may also need to move into SVGRenderBase as the RenderBox version depends
// on borderBoundingBox() which SVG RenderBox subclases (like SVGRenderBlock) do not implement.
IntRect RenderSVGModelObject::outlineBoundsForRepaint(RenderBoxModelObject* repaintContainer) const
{
    IntRect box = enclosingIntRect(repaintRectInLocalCoordinates());
    adjustRectForOutlineAndShadow(box);

    FloatQuad containerRelativeQuad = localToContainerQuad(FloatRect(box), repaintContainer);
    return containerRelativeQuad.enclosingBoundingBox();
}

void RenderSVGModelObject::absoluteRects(Vector<IntRect>& rects, int, int)
{
    rects.append(absoluteClippedOverflowRect());
}

void RenderSVGModelObject::absoluteQuads(Vector<FloatQuad>& quads)
{
    quads.append(absoluteClippedOverflowRect());
}

bool RenderSVGModelObject::nodeAtPoint(const HitTestRequest&, HitTestResult&, int, int, int, int, HitTestAction)
{
    ASSERT_NOT_REACHED();
    return false;
}

} // namespace WebCore

#endif // ENABLE(SVG)
