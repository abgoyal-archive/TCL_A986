

#include "config.h"
#include "BitmapInfo.h"

namespace WebCore {

BitmapInfo bitmapInfoForSize(int width, int height)
{
    BitmapInfo bitmapInfo;
    bitmapInfo.bmiHeader.biSize          = sizeof(BITMAPINFOHEADER);
    bitmapInfo.bmiHeader.biWidth         = width; 
    bitmapInfo.bmiHeader.biHeight        = height;
    bitmapInfo.bmiHeader.biPlanes        = 1;
    bitmapInfo.bmiHeader.biBitCount      = 32;
    bitmapInfo.bmiHeader.biCompression   = BI_RGB;
    bitmapInfo.bmiHeader.biSizeImage     = 0;
    bitmapInfo.bmiHeader.biXPelsPerMeter = 0;
    bitmapInfo.bmiHeader.biYPelsPerMeter = 0;
    bitmapInfo.bmiHeader.biClrUsed       = 0;
    bitmapInfo.bmiHeader.biClrImportant  = 0;

    return bitmapInfo;
}

BitmapInfo::BitmapInfo()
{
    memset(&bmiHeader, 0, sizeof(bmiHeader));
    bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
}

BitmapInfo BitmapInfo::create(const IntSize& size)
{
   return bitmapInfoForSize(size.width(), size.height());
}

BitmapInfo BitmapInfo::createBottomUp(const IntSize& size)
{
   return bitmapInfoForSize(size.width(), -size.height());
}

} // namespace WebCore
