

#ifndef TextCodecQt_h
#define TextCodecQt_h

#include "TextCodec.h"
#include "TextEncoding.h"
#include <QTextCodec>

namespace WebCore {

    class TextCodecQt : public TextCodec {
    public:
        static void registerEncodingNames(EncodingNameRegistrar);
        static void registerCodecs(TextCodecRegistrar);

        TextCodecQt(const TextEncoding&);
        virtual ~TextCodecQt();

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);

    private:
        TextEncoding m_encoding;
        QTextCodec *m_codec;
        QTextCodec::ConverterState m_state;
    };

} // namespace WebCore

#endif // TextCodecICU_h
