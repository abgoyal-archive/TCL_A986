

#include "config.h"
#include "DragImage.h"
#include "CachedImage.h"
#include "FloatSize.h"
#include "Image.h"

namespace WebCore {

IntSize dragImageSize(DragImageRef)
{
    return IntSize(0, 0);
}

void deleteDragImage(DragImageRef)
{
}

DragImageRef scaleDragImage(DragImageRef image, FloatSize)
{
    return image;
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
