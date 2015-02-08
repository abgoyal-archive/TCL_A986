

#ifndef TextEncodingRegistry_h
#define TextEncodingRegistry_h

#include <memory>
#include <wtf/PassOwnPtr.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

    class TextCodec;
    class TextEncoding;

    // Use TextResourceDecoder::decode to decode resources, since it handles BOMs.
    // Use TextEncoding::encode to encode, since it takes care of normalization.
    PassOwnPtr<TextCodec> newTextCodec(const TextEncoding&);

    // Only TextEncoding should use this function directly.
    const char* atomicCanonicalTextEncodingName(const char* alias);
    const char* atomicCanonicalTextEncodingName(const UChar* aliasCharacters, size_t aliasLength);

    // Only TextEncoding should use this function directly.
    bool noExtendedTextEncodingNameUsed();

}

#endif // TextEncodingRegistry_h
