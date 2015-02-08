

#include "config.h"
#include "SpaceSplitString.h"

#include <wtf/ASCIICType.h>

using namespace WTF;

namespace WebCore {

static bool hasNonASCIIOrUpper(const String& string)
{
    const UChar* characters = string.characters();
    unsigned length = string.length();
    bool hasUpper = false;
    UChar ored = 0;
    for (unsigned i = 0; i < length; i++) {
        UChar c = characters[i];
        hasUpper |= isASCIIUpper(c);
        ored |= c;
    }
    return hasUpper || (ored & ~0x7F);
}

void SpaceSplitStringData::createVector()
{
    ASSERT(!m_createdVector);
    ASSERT(m_vector.isEmpty());

    if (m_shouldFoldCase && hasNonASCIIOrUpper(m_string))
        m_string = m_string.foldCase();

    const UChar* characters = m_string.characters();
    unsigned length = m_string.length();
    unsigned start = 0;
    while (true) {
        while (start < length && isClassWhitespace(characters[start]))
            ++start;
        if (start >= length)
            break;
        unsigned end = start + 1;
        while (end < length && !isClassWhitespace(characters[end]))
            ++end;

        m_vector.append(AtomicString(characters + start, end - start));

        start = end + 1;
    }

    m_string = String();
    m_createdVector = true;
}

bool SpaceSplitStringData::containsAll(SpaceSplitStringData& other)
{
    ensureVector();
    other.ensureVector();
    size_t thisSize = m_vector.size();
    size_t otherSize = other.m_vector.size();
    for (size_t i = 0; i < otherSize; ++i) {
        const AtomicString& name = other.m_vector[i];
        size_t j;
        for (j = 0; j < thisSize; ++j) {
            if (m_vector[j] == name)
                break;
        }
        if (j == thisSize)
            return false;
    }
    return true;
}

} // namespace WebCore
