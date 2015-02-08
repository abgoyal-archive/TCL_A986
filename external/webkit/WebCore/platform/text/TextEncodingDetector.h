

#ifndef TextEncodingDetector_h
#define TextEncodingDetector_h

namespace WebCore {

    class TextEncoding;

    // Given a sequence of bytes in |data| of length |len| and an optional
    // hintEncodingName, detect the most likely character encoding. 
    // The way hintEncodingName is used is up to an implementation.
    // Currently, the only caller sets it to the parent frame encoding.
    bool detectTextEncoding(const char* data, size_t len,
                            const char* hintEncodingName,
                            TextEncoding* detectedEncoding);

} // namespace WebCore

#endif
