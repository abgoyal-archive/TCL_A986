

#ifndef WebGLUnsignedByteArray_h
#define WebGLUnsignedByteArray_h

#include "WebGLArray.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLUnsignedByteArray : public WebGLArray {
  public:
    virtual bool isUnsignedByteArray() const { return true; }

    static PassRefPtr<WebGLUnsignedByteArray> create(unsigned length);
    static PassRefPtr<WebGLUnsignedByteArray> create(unsigned char* array, unsigned length);
    static PassRefPtr<WebGLUnsignedByteArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    unsigned char* data() { return static_cast<unsigned char*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        if (value < std::numeric_limits<unsigned char>::min())
            value = std::numeric_limits<unsigned char>::min();
        else if (value > std::numeric_limits<unsigned char>::max())
            value = std::numeric_limits<unsigned char>::max();
        unsigned char* storage = static_cast<unsigned char*>(m_baseAddress);
        storage[index] = static_cast<unsigned char>(value);
    }

    bool get(unsigned index, unsigned char& result) const
    {
        if (index >= m_size)
            return false;
        result = item(index);
        return true;
    }

    unsigned char get(unsigned index) const
    {
        return item(index);
    }

    unsigned char item(unsigned index) const
    {
        ASSERT(index < m_size);
        unsigned char* storage = static_cast<unsigned char*>(m_baseAddress);
        return storage[index];
    }

    void set(WebGLUnsignedByteArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLUnsignedByteArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLUnsignedByteArray_h
