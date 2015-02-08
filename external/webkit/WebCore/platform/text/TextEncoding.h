

#ifndef TextEncoding_h
#define TextEncoding_h

#include "TextCodec.h"
#include <wtf/unicode/Unicode.h>

namespace WebCore {

    class CString;
    class String;

    class TextEncoding {
    public:
        TextEncoding() : m_name(0) { }
        TextEncoding(const char* name);
        TextEncoding(const String& name);

        bool isValid() const { return m_name; }
        const char* name() const { return m_name; }
        const char* domName() const; // name exposed via DOM
        bool usesVisualOrdering() const;
        bool isJapanese() const;
        
        PassRefPtr<StringImpl> displayString(PassRefPtr<StringImpl> str) const
        {
            if (m_backslashAsCurrencySymbol == '\\' || !str)
                return str;
            return str->replace('\\', m_backslashAsCurrencySymbol);
        }
        void displayBuffer(UChar* characters, unsigned len) const
        {
            if (m_backslashAsCurrencySymbol == '\\')
                return;
            for (unsigned i = 0; i < len; ++i) {
                if (characters[i] == '\\')
                    characters[i] = m_backslashAsCurrencySymbol;
            }
        }

        const TextEncoding& closestByteBasedEquivalent() const;
        const TextEncoding& encodingForFormSubmission() const;

        String decode(const char* str, size_t length) const
        {
            bool ignored;
            return decode(str, length, false, ignored);
        }
        String decode(const char*, size_t length, bool stopOnError, bool& sawError) const;
        CString encode(const UChar*, size_t length, UnencodableHandling) const;

    private:
        UChar backslashAsCurrencySymbol() const;
        bool isNonByteBasedEncoding() const;
        bool isUTF7Encoding() const;

        const char* m_name;
        UChar m_backslashAsCurrencySymbol;
    };

    inline bool operator==(const TextEncoding& a, const TextEncoding& b) { return a.name() == b.name(); }
    inline bool operator!=(const TextEncoding& a, const TextEncoding& b) { return a.name() != b.name(); }

    const TextEncoding& ASCIIEncoding();
    const TextEncoding& Latin1Encoding();
    const TextEncoding& UTF16BigEndianEncoding();
    const TextEncoding& UTF16LittleEndianEncoding();
    const TextEncoding& UTF32BigEndianEncoding();
    const TextEncoding& UTF32LittleEndianEncoding();
    const TextEncoding& UTF8Encoding();
    const TextEncoding& WindowsLatin1Encoding();

} // namespace WebCore

#endif // TextEncoding_h
