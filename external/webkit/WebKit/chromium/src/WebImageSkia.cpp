

#include "config.h"
#include "WebImage.h"

#include "Image.h"
#include "ImageSource.h"
#include "NativeImageSkia.h"
#include "SharedBuffer.h"

#include "WebData.h"
#include "WebSize.h"

#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

WebImage WebImage::fromData(const WebData& data, const WebSize& desiredSize)
{
    ImageSource source;
    source.setData(PassRefPtr<SharedBuffer>(data).get(), true);
    if (!source.isSizeAvailable())
        return WebImage();

    // Frames are arranged by decreasing size, then decreasing bit depth.
    // Pick the frame closest to |desiredSize|'s area without being smaller,
    // which has the highest bit depth.
    const size_t frameCount = source.frameCount();
    size_t index = 0;  // Default to first frame if none are large enough.
    int frameAreaAtIndex = 0;
    for (size_t i = 0; i < frameCount; ++i) {
        const IntSize frameSize = source.frameSizeAtIndex(i);
        if (WebSize(frameSize) == desiredSize) {
            index = i;
            break;  // Perfect match.
        }

        const int frameArea = frameSize.width() * frameSize.height();
        if (frameArea < (desiredSize.width * desiredSize.height))
            break;  // No more frames that are large enough.

        if (!i || (frameArea < frameAreaAtIndex)) {
            index = i;  // Closer to desired area than previous best match.
            frameAreaAtIndex = frameArea;
        }
    }

    OwnPtr<NativeImageSkia> frame(source.createFrameAtIndex(index));
    if (!frame.get())
        return WebImage();

    return WebImage(*frame);
}

void WebImage::reset()
{
    m_bitmap.reset();
}

void WebImage::assign(const WebImage& image)
{
    m_bitmap = image.m_bitmap;
}

bool WebImage::isNull() const
{
    return m_bitmap.isNull();
}

WebSize WebImage::size() const
{
    return WebSize(m_bitmap.width(), m_bitmap.height());
}

WebImage::WebImage(const PassRefPtr<Image>& image)
{
    operator=(image);
}

WebImage& WebImage::operator=(const PassRefPtr<Image>& image)
{
    NativeImagePtr p;
    if (image.get() && (p = image->nativeImageForCurrentFrame()))
        assign(*p);
    else
        reset();
    return *this;
}

} // namespace WebKit
