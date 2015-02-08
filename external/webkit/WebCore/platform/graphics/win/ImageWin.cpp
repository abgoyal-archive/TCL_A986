

#include "config.h"
#include "Image.h"
#include "BitmapImage.h"

#include "SharedBuffer.h"

// This function loads resources from WebKit
PassRefPtr<WebCore::SharedBuffer> loadResourceIntoBuffer(const char*);

namespace WebCore {

void BitmapImage::initPlatformData()
{
}

void BitmapImage::invalidatePlatformData()
{
}

PassRefPtr<Image> Image::loadPlatformResource(const char *name)
{
    RefPtr<SharedBuffer> buffer = loadResourceIntoBuffer(name);
    RefPtr<BitmapImage> img = BitmapImage::create();
    img->setData(buffer.release(), true);
    return img.release();
}

bool BitmapImage::getHBITMAP(HBITMAP bmp)
{
    return getHBITMAPOfSize(bmp, 0);
}

} // namespace WebCore
