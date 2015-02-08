

#ifndef WebGLByteArray_h
#define WebGLByteArray_h

#include "WebGLArray.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLArrayBuffer;

class WebGLByteArray : public WebGLArray {
  public:
    virtual bool isByteArray() const { return true; }

    static PassRefPtr<WebGLByteArray> create(unsigned length);
    static PassRefPtr<WebGLByteArray> create(signed char* array, unsigned length);
    static PassRefPtr<WebGLByteArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    char* data() { return static_cast<char*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        if (value < std::numeric_limits<signed char>::min())
            value = std::numeric_limits<signed char>::min();
        else if (value > std::numeric_limits<signed char>::max())
            value = std::numeric_limits<signed char>::max();
        signed char* storage = static_cast<signed char*>(m_baseAddress);
        storage[index] = static_cast<signed char>(value);
    }

    bool get(unsigned index, signed char& result) const
    {
        if (index >= m_size)
            return false;
        signed char* storage = static_cast<signed char*>(m_baseAddress);
        result = storage[index];
        return true;
    }

    signed char get(unsigned index) const
    {
        return item(index);
    }

    signed char item(unsigned index) const
    {
        ASSERT(index < m_size);
        signed char* storage = static_cast<signed char*>(m_baseAddress);
        return storage[index];
    }

    void set(WebGLByteArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLByteArray(PassRefPtr<WebGLArrayBuffer> buffer,
                   int offset,
                   unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLByteArray_h
