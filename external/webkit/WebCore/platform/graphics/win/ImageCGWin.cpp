

#include "config.h"
#include "Image.h"
#include "BitmapImage.h"
#include "BitmapInfo.h"
#include "GraphicsContext.h"
#include <ApplicationServices/ApplicationServices.h>

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

    RetainPtr<CGColorSpaceRef> deviceRGB(AdoptCF, CGColorSpaceCreateDeviceRGB());
    RetainPtr<CGContextRef> bitmapContext(AdoptCF, CGBitmapContextCreate(dibSection.dsBm.bmBits, dibSection.dsBm.bmWidth, dibSection.dsBm.bmHeight, 8,
        dibSection.dsBm.bmWidthBytes, deviceRGB.get(), kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst));

    // The BitmapImage takes ownership of this.
    CGImageRef cgImage = CGBitmapContextCreateImage(bitmapContext.get());

    return adoptRef(new BitmapImage(cgImage));
}

bool BitmapImage::getHBITMAPOfSize(HBITMAP bmp, LPSIZE size)
{
    ASSERT(bmp);

    BITMAP bmpInfo;
    GetObject(bmp, sizeof(BITMAP), &bmpInfo);

    ASSERT(bmpInfo.bmBitsPixel == 32);
    int bufferSize = bmpInfo.bmWidthBytes * bmpInfo.bmHeight;
    
    CGColorSpaceRef deviceRGB = CGColorSpaceCreateDeviceRGB();
    CGContextRef cgContext = CGBitmapContextCreate(bmpInfo.bmBits, bmpInfo.bmWidth, bmpInfo.bmHeight,
        8, bmpInfo.bmWidthBytes, deviceRGB, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
  
    GraphicsContext gc(cgContext);

    IntSize imageSize = BitmapImage::size();
    if (size)
        drawFrameMatchingSourceSize(&gc, FloatRect(0.0f, 0.0f, bmpInfo.bmWidth, bmpInfo.bmHeight), IntSize(*size), DeviceColorSpace, CompositeCopy);
    else
        draw(&gc, FloatRect(0.0f, 0.0f, bmpInfo.bmWidth, bmpInfo.bmHeight), FloatRect(0.0f, 0.0f, imageSize.width(), imageSize.height()), DeviceColorSpace, CompositeCopy);

    // Do cleanup
    CGContextRelease(cgContext);
    CGColorSpaceRelease(deviceRGB);

    return true;
}

void BitmapImage::drawFrameMatchingSourceSize(GraphicsContext* ctxt, const FloatRect& dstRect, const IntSize& srcSize, ColorSpace styleColorSpace, CompositeOperator compositeOp)
{
    size_t frames = frameCount();
    for (size_t i = 0; i < frames; ++i) {
        CGImageRef image = frameAtIndex(i);
        if (CGImageGetHeight(image) == static_cast<size_t>(srcSize.height()) && CGImageGetWidth(image) == static_cast<size_t>(srcSize.width())) {
            size_t currentFrame = m_currentFrame;
            m_currentFrame = i;
            draw(ctxt, dstRect, FloatRect(0.0f, 0.0f, srcSize.width(), srcSize.height()), styleColorSpace, compositeOp);
            m_currentFrame = currentFrame;
            return;
        }
    }

    // No image of the correct size was found, fallback to drawing the current frame
    IntSize imageSize = BitmapImage::size();
    draw(ctxt, dstRect, FloatRect(0.0f, 0.0f, imageSize.width(), imageSize.height()), styleColorSpace, compositeOp);
}

} // namespace WebCore
