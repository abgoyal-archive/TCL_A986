

#ifndef TextResourceDecoder_h
#define TextResourceDecoder_h

#include "TextEncoding.h"

namespace WebCore {

class TextResourceDecoder : public RefCounted<TextResourceDecoder> {
public:
    enum EncodingSource {
        DefaultEncoding,
        AutoDetectedEncoding,
        EncodingFromXMLHeader,
        EncodingFromMetaTag,
        EncodingFromCSSCharset,
        EncodingFromHTTPHeader,
        UserChosenEncoding,
        EncodingFromParentFrame
    };

    static PassRefPtr<TextResourceDecoder> create(const String& mimeType, const TextEncoding& defaultEncoding = TextEncoding(), bool usesEncodingDetector = false)
    {
        return adoptRef(new TextResourceDecoder(mimeType, defaultEncoding, usesEncodingDetector));
    }
    ~TextResourceDecoder();

    void setEncoding(const TextEncoding&, EncodingSource);
    const TextEncoding& encoding() const { return m_encoding; }

    String decode(const char* data, size_t length);
    String flush();

    void setHintEncoding(const TextResourceDecoder* hintDecoder)
    {
        // hintEncoding is for use with autodetection, which should be 
        // only invoked when hintEncoding comes from auto-detection.
        if (hintDecoder && hintDecoder->m_source == AutoDetectedEncoding)
            m_hintEncoding = hintDecoder->encoding().name();
    }
   
    void useLenientXMLDecoding() { m_useLenientXMLDecoding = true; }
    bool sawError() const { return m_sawError; }

private:
    TextResourceDecoder(const String& mimeType, const TextEncoding& defaultEncoding,
                        bool usesEncodingDetector);

    enum ContentType { PlainText, HTML, XML, CSS }; // PlainText only checks for BOM.
    static ContentType determineContentType(const String& mimeType);
    static const TextEncoding& defaultEncoding(ContentType, const TextEncoding& defaultEncoding);

    size_t checkForBOM(const char*, size_t);
    bool checkForCSSCharset(const char*, size_t, bool& movedDataToBuffer);
    bool checkForHeadCharset(const char*, size_t, bool& movedDataToBuffer);
    void detectJapaneseEncoding(const char*, size_t);
    bool shouldAutoDetect() const;

    ContentType m_contentType;
    TextEncoding m_encoding;
    OwnPtr<TextCodec> m_codec;
    EncodingSource m_source;
    const char* m_hintEncoding;
    Vector<char> m_buffer;
    bool m_checkedForBOM;
    bool m_checkedForCSSCharset;
    bool m_checkedForHeadCharset;
    bool m_useLenientXMLDecoding; // Don't stop on XML decoding errors.
    bool m_sawError;
    bool m_usesEncodingDetector;
};

}

#endif
