

#ifndef TextCodec_h
#define TextCodec_h

#include <memory>
#include <wtf/Noncopyable.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/Vector.h>
#include <wtf/unicode/Unicode.h>

#include "PlatformString.h"

namespace WebCore {
    class TextEncoding;

    // Specifies what will happen when a character is encountered that is
    // not encodable in the character set.
    enum UnencodableHandling {
        // Substitutes the replacement character "?".
        QuestionMarksForUnencodables,

        // Encodes the character as an XML entity. For example, U+06DE
        // would be "&#1758;" (0x6DE = 1758 in octal).
        EntitiesForUnencodables,

        // Encodes the character as en entity as above, but escaped
        // non-alphanumeric characters. This is used in URLs.
        // For example, U+6DE would be "%26%231758%3B".
        URLEncodedEntitiesForUnencodables,
    };

    typedef char UnencodableReplacementArray[32];

    class TextCodec : public Noncopyable {
    public:
        virtual ~TextCodec();

        String decode(const char* str, size_t length, bool flush = false)
        {
            bool ignored;
            return decode(str, length, flush, false, ignored);
        }
        
        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError) = 0;
        virtual CString encode(const UChar*, size_t length, UnencodableHandling) = 0;

        // Fills a null-terminated string representation of the given
        // unencodable character into the given replacement buffer. 
        // The length of the string (not including the null) will be returned.
        static int getUnencodableReplacement(unsigned codePoint, UnencodableHandling, UnencodableReplacementArray);
    };

    typedef void (*EncodingNameRegistrar)(const char* alias, const char* name);

    typedef PassOwnPtr<TextCodec> (*NewTextCodecFunction)(const TextEncoding&, const void* additionalData);
    typedef void (*TextCodecRegistrar)(const char* name, NewTextCodecFunction, const void* additionalData);

} // namespace WebCore

#endif // TextCodec_h
