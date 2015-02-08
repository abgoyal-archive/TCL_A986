

#ifndef CString_h
#define CString_h

#include "SharedBuffer.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

    class CStringBuffer : public RefCounted<CStringBuffer> {
    public:
        const char* data() { return m_vector.data(); }
        size_t length() { return m_vector.size(); }
        
    private:
        friend class CString;

        static PassRefPtr<CStringBuffer> create(unsigned length) { return adoptRef(new CStringBuffer(length)); }
        CStringBuffer(unsigned length) : m_vector(length) { }
        char* mutableData() { return m_vector.data(); }

        Vector<char> m_vector;
    };

    // A container for a null-terminated char array supporting copy-on-write
    // assignment.  The contained char array may be null.
    class CString {
    public:
        CString() { }
        CString(const char*);
        CString(const char*, unsigned length);
        CString(CStringBuffer* buffer) : m_buffer(buffer) { }
        static CString newUninitialized(size_t length, char*& characterBuffer);

        const char* data() const;
        char* mutableData();
        unsigned length() const;

        bool isNull() const { return !m_buffer; }

        CStringBuffer* buffer() const { return m_buffer.get(); }

    private:
        void copyBufferIfNeeded();
        void init(const char*, unsigned length);
        RefPtr<CStringBuffer> m_buffer;
    };

    bool operator==(const CString& a, const CString& b);
    inline bool operator!=(const CString& a, const CString& b) { return !(a == b); }

} // namespace WebCore

#endif // CString_h
