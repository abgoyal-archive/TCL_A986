

#include "config.h"
#include "DragImage.h"

#include "CachedImage.h"
#include "GraphicsContext.h"
#include "Image.h"
#include "RetainPtr.h"

#include <windows.h>

namespace WebCore {

IntSize dragImageSize(DragImageRef image)
{
    if (!image)
        return IntSize();
    BITMAP b;
    GetObject(image, sizeof(BITMAP), &b);
    return IntSize(b.bmWidth, b.bmHeight);
}

void deleteDragImage(DragImageRef image)
{
    if (image)
        ::DeleteObject(image);
}

DragImageRef dissolveDragImageToFraction(DragImageRef image, float)
{
    //We don't do this on windows as the dragimage is blended by the OS
    return image;
}
        
DragImageRef createDragImageIconForCachedImage(CachedImage* image)
{
    if (!image)
        return 0;

    String filename = image->response().suggestedFilename();
    
    SHFILEINFO shfi = {0};
    if (FAILED(SHGetFileInfo(static_cast<LPCWSTR>(filename.charactersWithNullTermination()), FILE_ATTRIBUTE_NORMAL,
        &shfi, sizeof(shfi), SHGFI_ICON | SHGFI_USEFILEATTRIBUTES)))
        return 0;

    ICONINFO iconInfo;
    if (!GetIconInfo(shfi.hIcon, &iconInfo)) {
        DestroyIcon(shfi.hIcon);
        return 0;
    }

    DestroyIcon(shfi.hIcon);
    DeleteObject(iconInfo.hbmMask);

    return iconInfo.hbmColor;
}
    
}
