

#ifndef TextCodecMac_h
#define TextCodecMac_h

#include "TextCodec.h"
#include <CoreServices/CoreServices.h>

namespace WebCore {

    typedef ::TextEncoding TECTextEncodingID;
    const TECTextEncodingID invalidEncoding = kCFStringEncodingInvalidId;

    class TextCodecMac : public TextCodec {
    public:
        static void registerEncodingNames(EncodingNameRegistrar);
        static void registerCodecs(TextCodecRegistrar);

        explicit TextCodecMac(TECTextEncodingID);
        virtual ~TextCodecMac();

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);

    private:
        OSStatus decode(const unsigned char* inputBuffer, int inputBufferLength, int& inputLength,
            void* outputBuffer, int outputBufferLength, int& outputLength);

        OSStatus createTECConverter() const;
        void releaseTECConverter() const;

        TECTextEncodingID m_encoding;
        UChar m_backslashAsCurrencySymbol;
        unsigned m_numBufferedBytes;
        unsigned char m_bufferedBytes[16]; // bigger than any single multi-byte character
        mutable TECObjectRef m_converterTEC;
    };

    struct TECConverterWrapper {
        TECConverterWrapper() : converter(0), encoding(invalidEncoding) { }
        ~TECConverterWrapper() { if (converter) TECDisposeConverter(converter); }

        TECObjectRef converter;
        TECTextEncodingID encoding;
    };

} // namespace WebCore

#endif // TextCodecMac_h
