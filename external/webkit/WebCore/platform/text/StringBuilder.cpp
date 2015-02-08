
 
#include "config.h"
#include "StringBuilder.h"

#include "StringBuffer.h"

namespace WebCore {

void StringBuilder::append(const String& string)
{
    if (string.isNull())
        return;

    if (m_totalLength == UINT_MAX)
        m_totalLength = string.length();
    else
        m_totalLength += string.length();

    if (!string.isEmpty())
        m_strings.append(string);
}

void StringBuilder::append(UChar c)
{
    if (m_totalLength == UINT_MAX)
        m_totalLength = 1;
    else
        m_totalLength += 1;

    m_strings.append(String(&c, 1));
}

void StringBuilder::append(char c)
{
    if (m_totalLength == UINT_MAX)
        m_totalLength = 1;
    else
        m_totalLength += 1;

    m_strings.append(String(&c, 1));
}

String StringBuilder::toString() const
{
    if (isNull())
        return String();

    unsigned count = m_strings.size();

    if (!count)
        return String(StringImpl::empty());
    if (count == 1)
        return m_strings[0];

    UChar* buffer;
    String result = String::createUninitialized(m_totalLength, buffer);

    UChar* p = buffer;
    for (unsigned i = 0; i < count; ++i) {
        StringImpl* string = m_strings[i].impl();
        unsigned length = string->length(); 
        memcpy(p, string->characters(), length * 2);
        p += length;
    }

    ASSERT(p == m_totalLength + buffer);

    return result;
}

void StringBuilder::clear()
{
    m_totalLength = UINT_MAX;
    m_strings.clear();
}

unsigned StringBuilder::length() const
{
    if (m_totalLength == UINT_MAX)
        return 0;
    return m_totalLength;
}

}
