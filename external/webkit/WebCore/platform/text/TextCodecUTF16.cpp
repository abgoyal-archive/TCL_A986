

#include "config.h"
#include "TextCodecUTF16.h"

#include "CString.h"
#include "PlatformString.h"
#include "StringBuffer.h"
#include <wtf/PassOwnPtr.h>

namespace WebCore {

void TextCodecUTF16::registerEncodingNames(EncodingNameRegistrar registrar)
{
    registrar("UTF-16LE", "UTF-16LE");
    registrar("UTF-16BE", "UTF-16BE");

    registrar("ISO-10646-UCS-2", "UTF-16LE");
    registrar("UCS-2", "UTF-16LE");
    registrar("UTF-16", "UTF-16LE");
    registrar("Unicode", "UTF-16LE");
    registrar("csUnicode", "UTF-16LE");
    registrar("unicodeFEFF", "UTF-16LE");

    registrar("unicodeFFFE", "UTF-16BE");
}

static PassOwnPtr<TextCodec> newStreamingTextDecoderUTF16LE(const TextEncoding&, const void*)
{
    return new TextCodecUTF16(true);
}

static PassOwnPtr<TextCodec> newStreamingTextDecoderUTF16BE(const TextEncoding&, const void*)
{
    return new TextCodecUTF16(false);
}

void TextCodecUTF16::registerCodecs(TextCodecRegistrar registrar)
{
    registrar("UTF-16LE", newStreamingTextDecoderUTF16LE, 0);
    registrar("UTF-16BE", newStreamingTextDecoderUTF16BE, 0);
}

String TextCodecUTF16::decode(const char* bytes, size_t length, bool, bool, bool&)
{
    if (!length)
        return String();

    const unsigned char* p = reinterpret_cast<const unsigned char*>(bytes);
    size_t numBytes = length + m_haveBufferedByte;
    size_t numChars = numBytes / 2;

    StringBuffer buffer(numChars);
    UChar* q = buffer.characters();

    if (m_haveBufferedByte) {
        UChar c;
        if (m_littleEndian)
            c = m_bufferedByte | (p[0] << 8);
        else
            c = (m_bufferedByte << 8) | p[0];
        *q++ = c;
        m_haveBufferedByte = false;
        p += 1;
        numChars -= 1;
    }

    if (m_littleEndian) {
        for (size_t i = 0; i < numChars; ++i) {
            UChar c = p[0] | (p[1] << 8);
            p += 2;
            *q++ = c;
        }
    } else {
        for (size_t i = 0; i < numChars; ++i) {
            UChar c = (p[0] << 8) | p[1];
            p += 2;
            *q++ = c;
        }
    }

    if (numBytes & 1) {
        ASSERT(!m_haveBufferedByte);
        m_haveBufferedByte = true;
        m_bufferedByte = p[0];
    }

    buffer.shrink(q - buffer.characters());

    return String::adopt(buffer);
}

CString TextCodecUTF16::encode(const UChar* characters, size_t length, UnencodableHandling)
{
    char* bytes;
    CString string = CString::newUninitialized(length * 2, bytes);

    // FIXME: CString is not a reasonable data structure for encoded UTF-16, which will have
    // null characters inside it. Perhaps the result of encode should not be a CString?
    if (m_littleEndian)
        for (size_t i = 0; i < length; ++i) {
            UChar c = characters[i];
            bytes[i * 2] = c;
            bytes[i * 2 + 1] = c >> 8;
        }
    else
        for (size_t i = 0; i < length; ++i) {
            UChar c = characters[i];
            bytes[i * 2] = c >> 8;
            bytes[i * 2 + 1] = c;
        }

    return string;
}

} // namespace WebCore
