

#include "config.h"
#include "Image.h"
#include "BitmapImage.h"
#include "GraphicsContext.h"
#include <cairo.h>
#include <cairo-win32.h>

#include <windows.h>
#include "PlatformString.h"

namespace WebCore {

PassRefPtr<BitmapImage> BitmapImage::create(HBITMAP hBitmap)
{
    DIBSECTION dibSection;
    if (!GetObject(hBitmap, sizeof(DIBSECTION), &dibSection))
        return 0;

    ASSERT(dibSection.dsBm.bmBitsPixel == 32);
    if (dibSection.dsBm.bmBitsPixel != 32)
        return 0;

    ASSERT(dibSection.dsBm.bmBits);
    if (!dibSection.dsBm.bmBits)
        return 0;

    cairo_surface_t* image = cairo_win32_surface_create_with_dib (CAIRO_FORMAT_ARGB32, dibSection.dsBm.bmWidth, dibSection.dsBm.bmHeight);

    // The BitmapImage object takes over ownership of the cairo_surface_t*, so no need to destroy here.
    return adoptRef(new BitmapImage(image));
}

bool BitmapImage::getHBITMAPOfSize(HBITMAP bmp, LPSIZE size)
{
    ASSERT(bmp);

    BITMAP bmpInfo;
    GetObject(bmp, sizeof(BITMAP), &bmpInfo);

    // If this is a 32bpp bitmap, which it always should be, we'll clear it so alpha-wise it will be visible
    if (bmpInfo.bmBitsPixel == 32) {
        int bufferSize = bmpInfo.bmWidthBytes * bmpInfo.bmHeight;
        memset(bmpInfo.bmBits, 255, bufferSize);
    }

    cairo_surface_t* image = cairo_image_surface_create_for_data((unsigned char*)bmpInfo.bmBits,
                                               CAIRO_FORMAT_ARGB32,
                                               bmpInfo.bmWidth,
                                               bmpInfo.bmHeight,
                                               bmpInfo.bmWidthBytes);


    cairo_t* targetRef = cairo_create(image);
    cairo_surface_destroy(image);

    GraphicsContext gc(targetRef);

    IntSize imageSize = BitmapImage::size();
    if (size)
        drawFrameMatchingSourceSize(&gc, FloatRect(0.0f, 0.0f, bmpInfo.bmWidth, bmpInfo.bmHeight), IntSize(*size), DeviceColorSpace, CompositeCopy);
    else
        draw(&gc, FloatRect(0.0f, 0.0f, bmpInfo.bmWidth, bmpInfo.bmHeight), FloatRect(0.0f, 0.0f, imageSize.width(), imageSize.height()), DeviceColorSpace, CompositeCopy);

    // Do cleanup
    cairo_destroy(targetRef);

    return true;
}

void BitmapImage::drawFrameMatchingSourceSize(GraphicsContext* ctxt, const FloatRect& dstRect, const IntSize& srcSize, ColorSpace styleColorSpace, CompositeOperator compositeOp)
{
    size_t frames = frameCount();
    for (size_t i = 0; i < frames; ++i) {
        cairo_surface_t* image = frameAtIndex(i);
        if (cairo_image_surface_get_height(image) == static_cast<size_t>(srcSize.height()) && cairo_image_surface_get_width(image) == static_cast<size_t>(srcSize.width())) {
            size_t currentFrame = m_currentFrame;
            m_currentFrame = i;
            draw(ctxt, dstRect, FloatRect(0.0f, 0.0f, srcSize.width(), srcSize.height()), DeviceColorSpace, compositeOp);
            m_currentFrame = currentFrame;
            return;
        }
    }

    // No image of the correct size was found, fallback to drawing the current frame
    IntSize imageSize = BitmapImage::size();
    draw(ctxt, dstRect, FloatRect(0.0f, 0.0f, imageSize.width(), imageSize.height()), DeviceColorSpace, compositeOp);
}

} // namespace WebCore
