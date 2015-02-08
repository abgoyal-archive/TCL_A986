

#include "config.h"
#include "WebImage.h"

#include "Image.h"
#include "ImageSource.h"
#include "SharedBuffer.h"

#include "WebData.h"
#include "WebSize.h"

#include <CoreGraphics/CGImage.h>

#include <wtf/PassRefPtr.h>
#include <wtf/RetainPtr.h>

using namespace WebCore;

namespace WebKit {

WebImage WebImage::fromData(const WebData& data, const WebSize& desiredSize)
{
    // FIXME: Do something like what WebImageSkia.cpp does to enumerate frames.
    // Not sure whether the CG decoder uses the same frame ordering rules (if so
    // we can just use the same logic).

    ImageSource source;
    source.setData(PassRefPtr<SharedBuffer>(data).get(), true);
    if (!source.isSizeAvailable())
        return WebImage();

    RetainPtr<CGImageRef> frame0(AdoptCF, source.createFrameAtIndex(0));
    if (!frame0)
        return WebImage();

    return WebImage(frame0.get());
}

void WebImage::reset()
{
    CGImageRelease(m_imageRef);
    m_imageRef = 0;
}

void WebImage::assign(const WebImage& image)
{
    assign(image.m_imageRef);
}

bool WebImage::isNull() const
{
    return !m_imageRef;
}

WebSize WebImage::size() const
{
    return WebSize(CGImageGetWidth(m_imageRef), CGImageGetHeight(m_imageRef));
}

WebImage::WebImage(const PassRefPtr<Image>& image)
    : m_imageRef(0)
{
    if (image.get())
        assign(image->nativeImageForCurrentFrame());
}

WebImage& WebImage::operator=(const PassRefPtr<Image>& image)
{
    if (image.get())
        assign(image->nativeImageForCurrentFrame());
    else
        reset();
    return *this;
}

void WebImage::assign(CGImageRef imageRef)
{
    // Make sure to retain the imageRef first incase m_imageRef == imageRef.
    CGImageRetain(imageRef);
    CGImageRelease(m_imageRef);
    m_imageRef = imageRef;
}

} // namespace WebKit
