

#include "config.h"
#include "RenderHTMLCanvas.h"

#include "Document.h"
#include "GraphicsContext.h"
#include "HTMLCanvasElement.h"
#include "HTMLNames.h"
#include "RenderView.h"
#include "FrameView.h"

namespace WebCore {

using namespace HTMLNames;

RenderHTMLCanvas::RenderHTMLCanvas(HTMLCanvasElement* element)
    : RenderReplaced(element, element->size())
{
    view()->frameView()->setIsVisuallyNonEmpty();
}

bool RenderHTMLCanvas::requiresLayer() const
{
    if (RenderReplaced::requiresLayer())
        return true;
    
#if ENABLE(3D_CANVAS)
    HTMLCanvasElement* canvas = static_cast<HTMLCanvasElement*>(node());
    return canvas && canvas->is3D();
#else
    return false;
#endif
}

void RenderHTMLCanvas::paintReplaced(PaintInfo& paintInfo, int tx, int ty)
{
    IntRect rect = contentBoxRect();
    rect.move(tx, ty);
    static_cast<HTMLCanvasElement*>(node())->paint(paintInfo.context, rect);
}

void RenderHTMLCanvas::canvasSizeChanged()
{
    IntSize canvasSize = static_cast<HTMLCanvasElement*>(node())->size();
    IntSize zoomedSize(canvasSize.width() * style()->effectiveZoom(), canvasSize.height() * style()->effectiveZoom());

    if (zoomedSize == intrinsicSize())
        return;

    setIntrinsicSize(zoomedSize);

    if (!parent())
        return;

    if (!prefWidthsDirty())
        setPrefWidthsDirty(true);

    IntSize oldSize = size();
    calcWidth();
    calcHeight();
    if (oldSize == size())
        return;

    if (!selfNeedsLayout())
        setNeedsLayout(true);
}

} // namespace WebCore
