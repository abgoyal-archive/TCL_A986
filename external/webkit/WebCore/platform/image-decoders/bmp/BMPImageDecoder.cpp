

#include "config.h"
#include "BMPImageDecoder.h"

#include "BMPImageReader.h"

namespace WebCore {

// Number of bits in .BMP used to store the file header (doesn't match
// "sizeof(BMPImageDecoder::BitmapFileHeader)" since we omit some fields and
// don't pack).
static const size_t sizeOfFileHeader = 14;

BMPImageDecoder::BMPImageDecoder()
    : ImageDecoder()
    , m_allDataReceived(false)
    , m_decodedOffset(0)
{
}

void BMPImageDecoder::setData(SharedBuffer* data, bool allDataReceived)
{
    if (failed())
        return;

    ImageDecoder::setData(data, allDataReceived);
    m_allDataReceived = allDataReceived;
    if (m_reader)
        m_reader->setData(data);
}

bool BMPImageDecoder::isSizeAvailable()
{
    if (!ImageDecoder::isSizeAvailable() && !failed())
        decodeWithCheckForDataEnded(true);

    return ImageDecoder::isSizeAvailable();
}

RGBA32Buffer* BMPImageDecoder::frameBufferAtIndex(size_t index)
{
    if (index)
        return 0;

    if (m_frameBufferCache.isEmpty())
        m_frameBufferCache.resize(1);

    RGBA32Buffer* buffer = &m_frameBufferCache.first();
    if (buffer->status() != RGBA32Buffer::FrameComplete && !failed())
        decodeWithCheckForDataEnded(false);
    return buffer;
}

void BMPImageDecoder::decodeWithCheckForDataEnded(bool onlySize)
{
    if (failed())
        return;

    // If we couldn't decode the image but we've received all the data, decoding
    // has failed.
    if (!decode(onlySize) && m_allDataReceived)
        setFailed();
}

bool BMPImageDecoder::decode(bool onlySize)
{
    size_t imgDataOffset = 0;
    if ((m_decodedOffset < sizeOfFileHeader)
        && !processFileHeader(&imgDataOffset))
        return false;

    if (!m_reader) {
        m_reader.set(new BMPImageReader(this, m_decodedOffset, imgDataOffset,
                                        false));
        m_reader->setData(m_data.get());
    }

    if (!m_frameBufferCache.isEmpty())
        m_reader->setBuffer(&m_frameBufferCache.first());

    return m_reader->decodeBMP(onlySize);
}

bool BMPImageDecoder::processFileHeader(size_t* imgDataOffset)
{
    ASSERT(imgDataOffset);

    // Read file header.
    ASSERT(!m_decodedOffset);
    if (m_data->size() < sizeOfFileHeader)
        return false;
    const uint16_t fileType =
        (m_data->data()[0] << 8) | static_cast<uint8_t>(m_data->data()[1]);
    *imgDataOffset = readUint32(10);
    m_decodedOffset = sizeOfFileHeader;

    // See if this is a bitmap filetype we understand.
    enum {
        BMAP = 0x424D,  // "BM"
        // The following additional OS/2 2.x header values (see
        // http://www.fileformat.info/format/os2bmp/egff.htm ) aren't widely
        // decoded, and are unlikely to be in much use.
        /*
        ICON = 0x4943,  // "IC"
        POINTER = 0x5054,  // "PT"
        COLORICON = 0x4349,  // "CI"
        COLORPOINTER = 0x4350,  // "CP"
        BITMAPARRAY = 0x4241,  // "BA"
        */
    };
    if (fileType != BMAP) {
        setFailed();
        return false;
    }

    return true;
}

} // namespace WebCore
