

#include "config.h"
#include "DragImage.h"

#include "NotImplemented.h"

namespace WebCore {

IntSize dragImageSize(DragImageRef image)
{
    notImplemented();
    return IntSize();
}

void deleteDragImage(DragImageRef image)
{
    notImplemented();
}

DragImageRef scaleDragImage(DragImageRef image, FloatSize scale)
{
    notImplemented();
    return 0;
}
    
DragImageRef dissolveDragImageToFraction(DragImageRef image, float)
{
    notImplemented();
    return image;
}
        
DragImageRef createDragImageFromImage(Image* img)
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
