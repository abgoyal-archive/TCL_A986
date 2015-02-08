

#ifndef StringBuffer_h
#define StringBuffer_h

#include <wtf/Assertions.h>
#include <wtf/Noncopyable.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

class StringBuffer : public Noncopyable {
public:
    explicit StringBuffer(unsigned length)
        : m_length(length)
        , m_data(static_cast<UChar*>(fastMalloc(length * sizeof(UChar))))
    {
    }
    ~StringBuffer()
    {
        fastFree(m_data);
    }

    void shrink(unsigned newLength)
    {
        ASSERT(newLength <= m_length);
        m_length = newLength;
    }

    void resize(unsigned newLength)
    {
        if (newLength > m_length)
            m_data = static_cast<UChar*>(fastRealloc(m_data, newLength * sizeof(UChar)));
        m_length = newLength;
    }

    unsigned length() const { return m_length; }
    UChar* characters() { return m_data; }

    UChar& operator[](unsigned i) { ASSERT(i < m_length); return m_data[i]; }

    UChar* release() { UChar* data = m_data; m_data = 0; return data; }

private:
    unsigned m_length;
    UChar* m_data;
};

}

#endif
