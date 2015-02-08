

#include "config.h"
#include "DragImage.h"

#include "CachedImage.h"
#include "Image.h"

#include "NotImplemented.h"


namespace WebCore {

IntSize dragImageSize(DragImageRef)
{
    notImplemented();
    return IntSize(0, 0);
}

void deleteDragImage(DragImageRef)
{
    notImplemented();
}

DragImageRef scaleDragImage(DragImageRef image, FloatSize)
{
    notImplemented();
    return image;
}

DragImageRef dissolveDragImageToFraction(DragImageRef image, float)
{
    notImplemented();
    return image;
}

DragImageRef createDragImageFromImage(Image*)
{
    notImplemented();
    return 0;
}

DragImageRef createDragImageIconForCachedImage(CachedImage*)
{
    notImplemented();
    return 0;
}

} // namespace WebCore

