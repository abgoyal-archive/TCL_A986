

#ifndef TextCodecUTF16_h
#define TextCodecUTF16_h

#include "TextCodec.h"

namespace WebCore {

    class TextCodecUTF16 : public TextCodec {
    public:
        static void registerEncodingNames(EncodingNameRegistrar);
        static void registerCodecs(TextCodecRegistrar);

        TextCodecUTF16(bool littleEndian) : m_littleEndian(littleEndian), m_haveBufferedByte(false) { }

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);

    private:
        bool m_littleEndian;
        bool m_haveBufferedByte;
        unsigned char m_bufferedByte;
    };

} // namespace WebCore

#endif // TextCodecUTF16_h
