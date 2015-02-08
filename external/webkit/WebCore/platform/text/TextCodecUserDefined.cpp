

#include "config.h"
#include "TextCodecUserDefined.h"

#include "CString.h"
#include "PlatformString.h"
#include "StringBuffer.h"
#include <stdio.h>
#include <wtf/PassOwnPtr.h>

namespace WebCore {

void TextCodecUserDefined::registerEncodingNames(EncodingNameRegistrar registrar)
{
    registrar("x-user-defined", "x-user-defined");
}

static PassOwnPtr<TextCodec> newStreamingTextDecoderUserDefined(const TextEncoding&, const void*)
{
    return new TextCodecUserDefined;
}

void TextCodecUserDefined::registerCodecs(TextCodecRegistrar registrar)
{
    registrar("x-user-defined", newStreamingTextDecoderUserDefined, 0);
}

String TextCodecUserDefined::decode(const char* bytes, size_t length, bool, bool, bool&)
{
    UChar* buffer;
    String result = String::createUninitialized(length, buffer);

    for (size_t i = 0; i < length; ++i) {
        signed char c = bytes[i];
        buffer[i] = c & 0xF7FF;
    }

    return result;
}

static CString encodeComplexUserDefined(const UChar* characters, size_t length, UnencodableHandling handling)
{
    Vector<char> result(length);
    char* bytes = result.data();

    size_t resultLength = 0;
    for (size_t i = 0; i < length; ) {
        UChar32 c;
        U16_NEXT(characters, i, length, c);
        signed char signedByte = c;
        if ((signedByte & 0xF7FF) == c)
            bytes[resultLength++] = signedByte;
        else {
            // No way to encode this character with x-user-defined.
            UnencodableReplacementArray replacement;
            int replacementLength = TextCodec::getUnencodableReplacement(c, handling, replacement);
            result.grow(resultLength + replacementLength + length - i);
            bytes = result.data();
            memcpy(bytes + resultLength, replacement, replacementLength);
            resultLength += replacementLength;
        }
    }

    return CString(bytes, resultLength);
}

CString TextCodecUserDefined::encode(const UChar* characters, size_t length, UnencodableHandling handling)
{
    char* bytes;
    CString string = CString::newUninitialized(length, bytes);

    // Convert the string a fast way and simultaneously do an efficient check to see if it's all ASCII.
    UChar ored = 0;
    for (size_t i = 0; i < length; ++i) {
        UChar c = characters[i];
        bytes[i] = c;
        ored |= c;
    }

    if (!(ored & 0xFF80))
        return string;

    // If it wasn't all ASCII, call the function that handles more-complex cases.
    return encodeComplexUserDefined(characters, length, handling);
}

} // namespace WebCore
