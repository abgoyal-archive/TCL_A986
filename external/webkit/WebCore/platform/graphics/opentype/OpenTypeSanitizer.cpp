

#include "config.h"
#if ENABLE(OPENTYPE_SANITIZER)
#include "OpenTypeSanitizer.h"

#include "SharedBuffer.h"
#include "opentype-sanitiser.h"
#include "ots-memory-stream.h"
#include <wtf/OwnArrayPtr.h>

namespace WebCore {

PassRefPtr<SharedBuffer> OpenTypeSanitizer::sanitize()
{
    if (!m_buffer)
        return 0;

    // This is the largest web font size which we'll try to transcode.
    static const size_t maxWebFontSize = 30 * 1024 * 1024; // 30 MB
    if (m_buffer->size() > maxWebFontSize)
        return 0;

    // A transcoded font is usually smaller than an original font.
    // However, it can be slightly bigger than the original one due to
    // name table replacement and/or padding for glyf table.
    static const size_t padLen = 20 * 1024; // 20 kB

    OwnArrayPtr<unsigned char> transcodeRawBuffer(new unsigned char[m_buffer->size() + padLen]);
    ots::MemoryStream output(transcodeRawBuffer.get(), m_buffer->size() + padLen);
    if (!ots::Process(&output, reinterpret_cast<const uint8_t*>(m_buffer->data()), m_buffer->size()))
        return 0;

    const size_t transcodeLen = output.Tell();
    return SharedBuffer::create(transcodeRawBuffer.get(), transcodeLen);
}

} // namespace WebCore

#endif // ENABLE(OPENTYPE_SANITIZER)
