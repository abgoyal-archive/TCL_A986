

#ifndef TextCodecLatin1_h
#define TextCodecLatin1_h

#include "TextCodec.h"

namespace WebCore {

    class TextCodecLatin1 : public TextCodec {
    public:
        static void registerEncodingNames(EncodingNameRegistrar);
        static void registerCodecs(TextCodecRegistrar);

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);
    };

} // namespace WebCore

#endif // TextCodecLatin1_h
