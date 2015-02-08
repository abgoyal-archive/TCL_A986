


#include "config.h"
#include "CString.h"

using std::min;

namespace WebCore {

CString::CString(const char* str)
{
    init(str, strlen(str));
}

CString::CString(const char* str, unsigned length)
{
    init(str, length);
}

void CString::init(const char* str, unsigned length)
{
    if (!str)
        return;
    
    m_buffer = CStringBuffer::create(length + 1);
    memcpy(m_buffer->mutableData(), str, length); 
    m_buffer->mutableData()[length] = '\0';
}

const char* CString::data() const
{
    return m_buffer ? m_buffer->data() : 0;
}

char* CString::mutableData()
{
    copyBufferIfNeeded();
    if (!m_buffer)
        return 0;
    return m_buffer->mutableData();
}
    
unsigned CString::length() const
{
    return m_buffer ? m_buffer->length() - 1 : 0;
}
    
CString CString::newUninitialized(size_t length, char*& characterBuffer)
{
    CString result;
    result.m_buffer = CStringBuffer::create(length + 1);
    char* bytes = result.m_buffer->mutableData();
    bytes[length] = '\0';
    characterBuffer = bytes;
    return result;
}

void CString::copyBufferIfNeeded()
{
    if (!m_buffer || m_buffer->hasOneRef())
        return;
        
    int len = m_buffer->length();
    RefPtr<CStringBuffer> m_temp = m_buffer;
    m_buffer = CStringBuffer::create(len);
    memcpy(m_buffer->mutableData(), m_temp->data(), len);
}

bool operator==(const CString& a, const CString& b)
{
    if (a.isNull() != b.isNull())
        return false;
    if (a.length() != b.length())
        return false;
    return !strncmp(a.data(), b.data(), min(a.length(), b.length()));
}

} // namespace WebCore
