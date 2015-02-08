

#ifndef TextCodecICU_h
#define TextCodecICU_h

#include "TextCodec.h"
#include "TextEncoding.h"

#include <unicode/utypes.h>

typedef struct UConverter UConverter;

namespace WebCore {

    class TextCodecICU : public TextCodec {
    public:
        static void registerBaseEncodingNames(EncodingNameRegistrar);
        static void registerBaseCodecs(TextCodecRegistrar);

        static void registerExtendedEncodingNames(EncodingNameRegistrar);
        static void registerExtendedCodecs(TextCodecRegistrar);

        TextCodecICU(const TextEncoding&);
        virtual ~TextCodecICU();

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);

    private:
        void createICUConverter() const;
        void releaseICUConverter() const;
        bool needsGBKFallbacks() const { return m_needsGBKFallbacks; }
        void setNeedsGBKFallbacks(bool needsFallbacks) { m_needsGBKFallbacks = needsFallbacks; }
        
        int decodeToBuffer(UChar* buffer, UChar* bufferLimit, const char*& source,
            const char* sourceLimit, int32_t* offsets, bool flush, UErrorCode& err);

        TextEncoding m_encoding;
        unsigned m_numBufferedBytes;
        unsigned char m_bufferedBytes[16]; // bigger than any single multi-byte character
        mutable UConverter* m_converterICU;
        mutable bool m_needsGBKFallbacks;
    };

    struct ICUConverterWrapper {
        ICUConverterWrapper()
            : converter(0)
        {
        }
        ~ICUConverterWrapper();

        UConverter* converter;
    };

} // namespace WebCore

#endif // TextCodecICU_h
