

#include "config.h"
#include "RenderReplica.h"

#include "RenderLayer.h"

namespace WebCore {

RenderReplica::RenderReplica(Node* n)
: RenderBox(n)
{
    // This is a hack. Replicas are synthetic, and don't pick up the attributes of the
    // renderers being replicated, so they always report that they are inline, non-replaced.
    // However, we need transforms to be applied to replicas for reflections, so have to pass
    // the if (!isInline() || isReplaced()) check before setHasTransform().
    setReplaced(true);
}

RenderReplica::~RenderReplica()
{}
    
void RenderReplica::layout()
{
    setFrameRect(parentBox()->borderBoxRect());
    setNeedsLayout(false);
}

void RenderReplica::calcPrefWidths()
{
    m_minPrefWidth = parentBox()->width();
    m_maxPrefWidth = m_minPrefWidth;
    setPrefWidthsDirty(false);
}

void RenderReplica::paint(PaintInfo& paintInfo, int tx, int ty)
{
    if (paintInfo.phase != PaintPhaseForeground && paintInfo.phase != PaintPhaseMask)
        return;
 
    tx += x();
    ty += y();

    if (paintInfo.phase == PaintPhaseForeground)
        // Turn around and paint the parent layer. Use temporary clipRects, so that the layer doesn't end up caching clip rects
        // computing using the wrong rootLayer
        layer()->parent()->paintLayer(layer()->transform() ? layer()->parent() : layer()->enclosingTransformedAncestor(),
                                      paintInfo.context, paintInfo.rect,
                                      PaintBehaviorNormal, 0, 0,
                                      RenderLayer::PaintLayerHaveTransparency | RenderLayer::PaintLayerAppliedTransform | RenderLayer::PaintLayerTemporaryClipRects | RenderLayer::PaintLayerPaintingReflection);
    else if (paintInfo.phase == PaintPhaseMask)
        paintMask(paintInfo, tx, ty);
}

} // namespace WebCore
