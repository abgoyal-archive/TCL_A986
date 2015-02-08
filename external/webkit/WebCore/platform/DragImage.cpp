

#include "config.h"
#include "DragImage.h"

#if ENABLE(DRAG_SUPPORT)
#include "DragController.h"

#include "Frame.h"

namespace WebCore {
    
DragImageRef fitDragImageToMaxSize(DragImageRef image, const IntSize& srcSize, const IntSize& size)
{
    float heightResizeRatio = 0.0f;
    float widthResizeRatio = 0.0f;
    float resizeRatio = -1.0f;
    IntSize originalSize = dragImageSize(image);
    
    if (srcSize.width() > size.width()) {
        widthResizeRatio = size.width() / (float)srcSize.width();
        resizeRatio = widthResizeRatio;
    }
    
    if (srcSize.height() > size.height()) {
        heightResizeRatio = size.height() / (float)srcSize.height();
        if ((resizeRatio < 0.0f) || (resizeRatio > heightResizeRatio))
            resizeRatio = heightResizeRatio;
    }
    
    if (srcSize == originalSize)
        return resizeRatio > 0.0f ? scaleDragImage(image, FloatSize(resizeRatio, resizeRatio)) : image;
    
    // The image was scaled in the webpage so at minimum we must account for that scaling
    float scalex = srcSize.width() / (float)originalSize.width();
    float scaley = srcSize.height() / (float)originalSize.height();
    if (resizeRatio > 0.0f) {
        scalex *= resizeRatio;
        scaley *= resizeRatio;
    }
    
    return scaleDragImage(image, FloatSize(scalex, scaley));
}
    
DragImageRef createDragImageForSelection(Frame* frame)
{
    DragImageRef image = frame->dragImageForSelection();
    if (image)
        dissolveDragImageToFraction(image, DragController::DragImageAlpha);
    return image;
}

} // namespace WebCore

#endif // ENABLE(DRAG_SUPPORT)
