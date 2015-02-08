

#include "config.h"
#include "TextStream.h"

#include "PlatformString.h"
#include <wtf/StringExtras.h>

namespace WebCore {

static const size_t printBufferSize = 100; // large enough for any integer or floating point value in string format, including trailing null character

TextStream& TextStream::operator<<(bool b)
{
    return *this << (b ? "1" : "0");
}

TextStream& TextStream::operator<<(int i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%d", i);
    return *this << buffer;
}

TextStream& TextStream::operator<<(unsigned i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%u", i);
    return *this << buffer;
}

TextStream& TextStream::operator<<(long i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%ld", i);
    return *this << buffer;
}

TextStream& TextStream::operator<<(unsigned long i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%lu", i);
    return *this << buffer;
}

TextStream& TextStream::operator<<(float f)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%.2f", f);
    return *this << buffer;
}

TextStream& TextStream::operator<<(double d)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%.2f", d);
    return *this << buffer;
}

TextStream& TextStream::operator<<(const char* string)
{
    size_t stringLength = strlen(string);
    size_t textLength = m_text.size();
    m_text.grow(textLength + stringLength);
    for (size_t i = 0; i < stringLength; ++i)
        m_text[textLength + i] = string[i];
    return *this;
}

TextStream& TextStream::operator<<(void* p)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%p", p);
    return *this << buffer;
}

TextStream& TextStream::operator<<(const String& string)
{
    append(m_text, string);
    return *this;
}

String TextStream::release()
{
    return String::adopt(m_text);
}

#if OS(WINDOWS) && PLATFORM(X86_64) && COMPILER(MSVC)
TextStream& TextStream::operator<<(__int64 i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%I64i", i);
    return *this << buffer;
}
TextStream& TextStream::operator<<(unsigned __int64 i)
{
    char buffer[printBufferSize];
    snprintf(buffer, sizeof(buffer) - 1, "%I64u", i);
    return *this << buffer;
}
#endif

}
