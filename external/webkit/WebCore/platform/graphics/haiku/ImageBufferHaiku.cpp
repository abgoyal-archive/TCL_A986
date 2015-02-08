

#include "config.h"
#include "ImageBuffer.h"

#include "GraphicsContext.h"
#include "ImageData.h"
#include "NotImplemented.h"


namespace WebCore {

ImageBufferData::ImageBufferData(const IntSize&)
{
}

ImageBuffer::ImageBuffer(const IntSize&, ImageColorSpace imageColorSpace, bool& success)
    : m_data(IntSize())
{
    notImplemented();
    success = false;
}

ImageBuffer::~ImageBuffer()
{
}

GraphicsContext* ImageBuffer::context() const
{
    notImplemented();
    return 0;
}

PassRefPtr<ImageData> ImageBuffer::getUnmultipliedImageData(const IntRect& rect) const
{
    notImplemented();
    return 0;
}

PassRefPtr<ImageData> ImageBuffer::getPremultipliedImageData(const IntRect& rect) const
{
    notImplemented();
    return 0;
}

void ImageBuffer::putUnmultipliedImageData(ImageData* source, const IntRect& sourceRect, const IntPoint& destPoint)
{
    notImplemented();
}

void ImageBuffer::putPremultipliedImageData(ImageData* source, const IntRect& sourceRect, const IntPoint& destPoint)
{
    notImplemented();
}

String ImageBuffer::toDataURL(const String&) const
{
    notImplemented();
    return String();
}

Image* ImageBuffer::image() const
{
    notImplemented();
    return 0;
}

void ImageBuffer::platformTransformColorSpace(const Vector<int>& lookUpTable)
{
    notImplemented();
}

} // namespace WebCore

