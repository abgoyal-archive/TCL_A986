

#include "config.h"
#include "ImageDecoder.h"
#if PLATFORM(ANDROID)
#include "SkBitmapRef.h"
#endif

namespace WebCore {

RGBA32Buffer::RGBA32Buffer()
    : m_status(FrameEmpty)
    , m_duration(0)
    , m_disposalMethod(DisposeNotSpecified)
{
}

void RGBA32Buffer::clear()
{
    m_bitmap.reset();
    m_status = FrameEmpty;
    // NOTE: Do not reset other members here; clearFrameBufferCache()
    // calls this to free the bitmap data, but other functions like
    // initFrameBuffer() and frameComplete() may still need to read
    // other metadata out of this frame later.
}

void RGBA32Buffer::zeroFill()
{
    m_bitmap.eraseARGB(0, 0, 0, 0);
}

void RGBA32Buffer::copyBitmapData(const RGBA32Buffer& other)
{
    if (this == &other)
        return;

    m_bitmap.reset();
    const NativeImageSkia& otherBitmap = other.m_bitmap;
    otherBitmap.copyTo(&m_bitmap, otherBitmap.config());
}

bool RGBA32Buffer::setSize(int newWidth, int newHeight)
{
    // This function should only be called once, it will leak memory
    // otherwise.
    ASSERT(width() == 0 && height() == 0);
    m_bitmap.setConfig(SkBitmap::kARGB_8888_Config, newWidth, newHeight);
    if (!m_bitmap.allocPixels())
        return false;

    // Zero the image.
    zeroFill();

    return true;
}

NativeImagePtr RGBA32Buffer::asNewNativeImage() const
{
#if PLATFORM(ANDROID)
    return new SkBitmapRef(m_bitmap);
#else
    return new NativeImageSkia(m_bitmap);
#endif
}

bool RGBA32Buffer::hasAlpha() const
{
    return !m_bitmap.isOpaque();
}

void RGBA32Buffer::setHasAlpha(bool alpha)
{
    m_bitmap.setIsOpaque(!alpha);
}

void RGBA32Buffer::setStatus(FrameStatus status)
{
    m_status = status;
    if (m_status == FrameComplete)
        m_bitmap.setDataComplete();  // Tell the bitmap it's done.
}

RGBA32Buffer& RGBA32Buffer::operator=(const RGBA32Buffer& other)
{
    if (this == &other)
        return *this;

    m_bitmap = other.m_bitmap;
    // Keep the pixels locked since we will be writing directly into the
    // bitmap throughout this object's lifetime.
    m_bitmap.lockPixels();
    setRect(other.rect());
    setStatus(other.status());
    setDuration(other.duration());
    setDisposalMethod(other.disposalMethod());
    return *this;
}

int RGBA32Buffer::width() const
{
    return m_bitmap.width();
}

int RGBA32Buffer::height() const
{
    return m_bitmap.height();
}

} // namespace WebCore
