

#include "config.h"
#include "ImageDecoder.h"

#include <QPixmap>
#include <stdio.h>

namespace WebCore {

RGBA32Buffer::RGBA32Buffer()
    : m_hasAlpha(false) 
    , m_size()
    , m_status(FrameEmpty)
    , m_duration(0)
    , m_disposalMethod(DisposeNotSpecified)
{
}

// The image must not have format 8888 pre multiplied...
void RGBA32Buffer::setDecodedImage(const QImage& image)
{
    m_image = image;
    m_size = image.size();
    m_hasAlpha = image.hasAlphaChannel();
}

void RGBA32Buffer::clear()
{
    m_image = QImage();
    m_status = FrameEmpty;
    // NOTE: Do not reset other members here; clearFrameBufferCache()
    // calls this to free the bitmap data, but other functions like
    // initFrameBuffer() and frameComplete() may still need to read
    // other metadata out of this frame later.
}

void RGBA32Buffer::zeroFill()
{
    m_image.fill(0);
}

void RGBA32Buffer::copyBitmapData(const RGBA32Buffer& other)
{
    if (this == &other)
        return;

    m_image = other.m_image;
    m_size = other.m_size;
    m_hasAlpha = other.m_hasAlpha;
}

bool RGBA32Buffer::setSize(int newWidth, int newHeight)
{
    // This function should only be called once, it will leak memory
    // otherwise.
    ASSERT(width() == 0 && height() == 0);

    m_size = IntSize(newWidth, newHeight);
    m_image = QImage(newWidth, newHeight, QImage::Format_ARGB32_Premultiplied);
    if (m_image.isNull())
        return false;

    // Zero the image.
    zeroFill();

    return true;
}

QPixmap* RGBA32Buffer::asNewNativeImage() const
{
    QPixmap pix = QPixmap::fromImage(m_image);
    m_image = QImage();

    return new QPixmap(pix);
}

bool RGBA32Buffer::hasAlpha() const
{
    return m_hasAlpha;
}

void RGBA32Buffer::setHasAlpha(bool alpha)
{
    m_hasAlpha = alpha;
}

void RGBA32Buffer::setStatus(FrameStatus status)
{
    m_status = status;
}

RGBA32Buffer& RGBA32Buffer::operator=(const RGBA32Buffer& other)
{
    if (this == &other)
        return *this;

    copyBitmapData(other);
    setRect(other.rect());
    setStatus(other.status());
    setDuration(other.duration());
    setDisposalMethod(other.disposalMethod());
    return *this;
}

int RGBA32Buffer::width() const
{
    return m_size.width();
}

int RGBA32Buffer::height() const
{
    return m_size.height();
}

}
