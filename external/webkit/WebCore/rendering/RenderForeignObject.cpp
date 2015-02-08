

#include "config.h"

#if ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)
#include "RenderForeignObject.h"

#include "GraphicsContext.h"
#include "RenderView.h"
#include "SVGForeignObjectElement.h"
#include "SVGLength.h"
#include "SVGRenderSupport.h"
#include "SVGTransformList.h"

namespace WebCore {

RenderForeignObject::RenderForeignObject(SVGForeignObjectElement* node) 
    : RenderSVGBlock(node)
{
}

FloatPoint RenderForeignObject::translationForAttributes() const
{
    SVGForeignObjectElement* foreign = static_cast<SVGForeignObjectElement*>(node());
    return FloatPoint(foreign->x().value(foreign), foreign->y().value(foreign));
}

void RenderForeignObject::paint(PaintInfo& paintInfo, int, int)
{
    if (paintInfo.context->paintingDisabled())
        return;

    // Copy the paint info so that modifications to the damage rect do not affect callers
    PaintInfo childPaintInfo = paintInfo;
    childPaintInfo.context->save();
    applyTransformToPaintInfo(childPaintInfo, localToParentTransform());
    childPaintInfo.context->clip(clipRect(0, 0));

    float opacity = style()->opacity();
    if (opacity < 1.0f)
        childPaintInfo.context->beginTransparencyLayer(opacity);

    RenderBlock::paint(childPaintInfo, 0, 0);

    if (opacity < 1.0f)
        childPaintInfo.context->endTransparencyLayer();

    childPaintInfo.context->restore();
}

FloatRect RenderForeignObject::objectBoundingBox() const
{
    return borderBoxRect();
}

FloatRect RenderForeignObject::repaintRectInLocalCoordinates() const
{
    // HACK: to maintain historical LayoutTest results for now.
    // RenderForeignObject is a RenderBlock (not a RenderSVGModelObject) so this
    // should not affect repaint correctness.  But it should really be:
    // return borderBoxRect();
    return FloatRect();
}

void RenderForeignObject::computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect& rect, bool fixed)
{
    rect = localToParentTransform().mapRect(rect);
    style()->svgStyle()->inflateForShadow(rect);
    RenderBlock::computeRectForRepaint(repaintContainer, rect, fixed);
}

const AffineTransform& RenderForeignObject::localToParentTransform() const
{
    FloatPoint attributeTranslation(translationForAttributes());
    m_localToParentTransform = localTransform().translateRight(attributeTranslation.x(), attributeTranslation.y());
    return m_localToParentTransform;
}

void RenderForeignObject::layout()
{
    ASSERT(needsLayout());
    ASSERT(!view()->layoutStateEnabled()); // RenderSVGRoot disables layoutState for the SVG rendering tree.

    LayoutRepainter repainter(*this, checkForRepaintDuringLayout());
    m_localTransform = static_cast<SVGForeignObjectElement*>(node())->animatedLocalTransform();

    RenderBlock::layout();
    repainter.repaintAfterLayout();

    setNeedsLayout(false);
}

bool RenderForeignObject::nodeAtFloatPoint(const HitTestRequest& request, HitTestResult& result, const FloatPoint& pointInParent, HitTestAction hitTestAction)
{
    FloatPoint localPoint = localToParentTransform().inverse().mapPoint(pointInParent);
    return RenderBlock::nodeAtPoint(request, result, static_cast<int>(localPoint.x()), static_cast<int>(localPoint.y()), 0, 0, hitTestAction);
}

bool RenderForeignObject::nodeAtPoint(const HitTestRequest&, HitTestResult&, int, int, int, int, HitTestAction)
{
    ASSERT_NOT_REACHED();
    return false;
}

void RenderForeignObject::mapLocalToContainer(RenderBoxModelObject* repaintContainer, bool fixed , bool useTransforms, TransformState& transformState) const
{
    SVGRenderBase::mapLocalToContainer(this, repaintContainer, fixed, useTransforms, transformState);
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)
