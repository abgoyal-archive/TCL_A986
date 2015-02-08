

#ifndef WebGLUnsignedIntArray_h
#define WebGLUnsignedIntArray_h

#include "WebGLArray.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLUnsignedIntArray : public WebGLArray {
  public:
    virtual bool isUnsignedIntArray() const { return true; }

    static PassRefPtr<WebGLUnsignedIntArray> create(unsigned length);
    static PassRefPtr<WebGLUnsignedIntArray> create(unsigned int* array, unsigned length);
    static PassRefPtr<WebGLUnsignedIntArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    unsigned int* data() { return static_cast<unsigned int*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        if (value < std::numeric_limits<unsigned int>::min())
            value = std::numeric_limits<unsigned int>::min();
        else if (value > std::numeric_limits<unsigned int>::max())
            value = std::numeric_limits<unsigned int>::max();
        unsigned int* storage = static_cast<unsigned int*>(m_baseAddress);
        storage[index] = static_cast<unsigned int>(value);
    }

    bool get(unsigned index, unsigned int& result) const
    {
        if (index >= m_size)
            return false;
        result = item(index);
        return true;
    }

    unsigned int get(unsigned index) const
    {
        return item(index);
    }

    unsigned int item(unsigned index) const
    {
        ASSERT(index < m_size);
        unsigned int* storage = static_cast<unsigned int*>(m_baseAddress);
        return storage[index];
    }

    void set(WebGLUnsignedIntArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLUnsignedIntArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLUnsignedIntArray_h
