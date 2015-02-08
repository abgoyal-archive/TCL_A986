

#ifndef WebGLIntArray_h
#define WebGLIntArray_h

#include "WebGLArray.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLIntArray : public WebGLArray {
  public:
    virtual bool isIntArray() const { return true; }

    static PassRefPtr<WebGLIntArray> create(unsigned length);
    static PassRefPtr<WebGLIntArray> create(int* array, unsigned length);
    static PassRefPtr<WebGLIntArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    int* data() { return static_cast<int*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        if (value < std::numeric_limits<int>::min())
            value = std::numeric_limits<int>::min();
        else if (value > std::numeric_limits<int>::max())
            value = std::numeric_limits<int>::max();
        int* storage = static_cast<int*>(m_baseAddress);
        storage[index] = static_cast<int>(value);
    }

    bool get(unsigned index, int& result) const
    {
        if (index >= m_size)
            return false;
        result = item(index);
        return true;
    }

    int get(unsigned index) const
    {
        return item(index);
    }

    int item(unsigned index) const
    {
        ASSERT(index < m_size);
        int* storage = static_cast<int*>(m_baseAddress);
        return storage[index];
    }

    void set(WebGLIntArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLIntArray(PassRefPtr<WebGLArrayBuffer> buffer,
                  int byteOffset,
                  unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLIntArray_h
