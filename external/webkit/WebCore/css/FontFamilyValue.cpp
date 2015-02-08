

#include "config.h"
#include "FontFamilyValue.h"

namespace WebCore {

// FIXME: This appears identical to isCSSTokenizerIdentifier from CSSPrimitiveValue.cpp, we should use a single function.
static bool isValidCSSIdentifier(const String& string)
{
    unsigned length = string.length();
    if (!length)
        return false;

    const UChar* characters = string.characters();
    UChar c = characters[0];
    if (!(c == '_' || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-' || c >= 0x80))
        return false;

    for (unsigned i = 1; i < length; ++i) {
        c = characters[i];
        if (!(c == '_' || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c >= 0x80))
            return false;
    }

    return true;
}

// Quotes the string if it needs quoting.
// We use single quotes because serialization code uses double quotes, and it's nice to
// avoid having to turn all the quote marks into &quot; as we would have to.
static String quoteStringIfNeeded(const String& string)
{
    if (isValidCSSIdentifier(string))
        return string;

    // FIXME: Also need to transform control characters (00-1F) into \ sequences.
    // FIXME: This is inefficient -- should use a Vector<UChar> instead.
    String quotedString = string;
    quotedString.replace('\\', "\\\\");
    quotedString.replace('\'', "\\'");
    return "'" + quotedString + "'";
}

FontFamilyValue::FontFamilyValue(const String& familyName)
    : CSSPrimitiveValue(String(), CSS_STRING)
    , m_familyName(familyName)
{
    // If there is anything in parentheses or square brackets at the end, delete it.
    // FIXME: Do we really need this? The original code mentioned "a language tag in
    // braces at the end" and "[Xft] qualifiers", but it's not clear either of those
    // is in active use on the web.
    unsigned length = m_familyName.length();
    while (length >= 3) {
        UChar startCharacter = 0;
        switch (m_familyName[length - 1]) {
            case ']':
                startCharacter = '[';
                break;
            case ')':
                startCharacter = '(';
                break;
        }
        if (!startCharacter)
            break;
        unsigned first = 0;
        for (unsigned i = length - 2; i > 0; --i) {
            if (m_familyName[i - 1] == ' ' && m_familyName[i] == startCharacter)
                first = i - 1;
        }
        if (!first)
            break;
        length = first;
    }
    m_familyName.truncate(length);
}

void FontFamilyValue::appendSpaceSeparated(const UChar* characters, unsigned length)
{
    m_familyName.append(' ');
    m_familyName.append(characters, length);
}

String FontFamilyValue::cssText() const
{
    return quoteStringIfNeeded(m_familyName);
}

}
