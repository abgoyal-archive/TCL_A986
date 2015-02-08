

#include "config.h"
#include "DragImage.h"

#include "CachedImage.h"
#include "GraphicsContext.h"
#include "Image.h"

#include <windows.h>

namespace WebCore {

IntSize dragImageSize(DragImageRef)
{
    return IntSize(0, 0);
}

void deleteDragImage(DragImageRef image)
{
    if (image)
        ::DeleteObject(image);
}

DragImageRef scaleDragImage(DragImageRef, FloatSize)
{
    return 0;
}

DragImageRef dissolveDragImageToFraction(DragImageRef image, float)
{
    return image;
}

DragImageRef createDragImageFromImage(Image*)
{
    return 0;
}

DragImageRef createDragImageIconForCachedImage(CachedImage*)
{
    return 0;
}

}
