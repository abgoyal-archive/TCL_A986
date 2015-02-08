

#include "config.h"
#include "FrameWin.h"

#include <windows.h>

#include "BitmapInfo.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "Settings.h"

#include <CoreGraphics/CoreGraphics.h>

using std::min;

namespace WebCore {

static void drawRectIntoContext(IntRect rect, FrameView* view, GraphicsContext* gc)
{
    IntSize offset = view->scrollOffset();
    rect.move(-offset.width(), -offset.height());
    rect = view->convertToContainingWindow(rect);

    gc->concatCTM(AffineTransform().translate(-rect.x(), -rect.y()));

    view->paint(gc, rect);
}

static HBITMAP imageFromRect(const Frame* frame, IntRect& ir)
{
    PaintBehavior oldPaintBehavior = frame->view()->paintBehavior();
    frame->view()->setPaintBehavior(oldPaintBehavior | PaintBehaviorFlattenCompositingLayers);

    void* bits;
    HDC hdc = CreateCompatibleDC(0);
    int w = ir.width();
    int h = ir.height();
    BitmapInfo bmp = BitmapInfo::create(IntSize(w, h));

    HBITMAP hbmp = CreateDIBSection(0, &bmp, DIB_RGB_COLORS, static_cast<void**>(&bits), 0, 0);
    HBITMAP hbmpOld = static_cast<HBITMAP>(SelectObject(hdc, hbmp));
    CGColorSpaceRef deviceRGB = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(static_cast<void*>(bits), w, h,
        8, w * sizeof(RGBQUAD), deviceRGB, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGColorSpaceRelease(deviceRGB);
    CGContextSaveGState(context);

    GraphicsContext gc(context);

    drawRectIntoContext(ir, frame->view(), &gc);

    CGContextRelease(context);
    SelectObject(hdc, hbmpOld);
    DeleteDC(hdc);

    frame->view()->setPaintBehavior(oldPaintBehavior);

    return hbmp;
}

HBITMAP imageFromSelection(Frame* frame, bool forceBlackText)
{
    frame->document()->updateLayout();

    frame->view()->setPaintBehavior(PaintBehaviorSelectionOnly | (forceBlackText ? PaintBehaviorForceBlackText : 0));
    FloatRect fr = frame->selectionBounds();
    IntRect ir(static_cast<int>(fr.x()), static_cast<int>(fr.y()),
               static_cast<int>(fr.width()), static_cast<int>(fr.height()));
    HBITMAP image = imageFromRect(frame, ir);
    frame->view()->setPaintBehavior(PaintBehaviorNormal);
    return image;
}

HBITMAP Frame::nodeImage(Node* node) const
{
    RenderObject* renderer = node->renderer();
    if (!renderer)
        return 0;

    IntRect topLevelRect;
    IntRect paintingRect = renderer->paintingRootRect(topLevelRect);

    document()->updateLayout();

    m_view->setNodeToDraw(node); // invoke special sub-tree drawing mode
    HBITMAP result = imageFromRect(this, paintingRect);
    m_view->setNodeToDraw(0);

    return result;
}

} // namespace WebCore
