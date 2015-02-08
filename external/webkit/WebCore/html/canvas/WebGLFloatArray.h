

#ifndef WebGLFloatArray_h
#define WebGLFloatArray_h

#include "WebGLArray.h"
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLFloatArray : public WebGLArray {
  public:
    virtual bool isFloatArray() const { return true; }

    static PassRefPtr<WebGLFloatArray> create(unsigned length);
    static PassRefPtr<WebGLFloatArray> create(float* array, unsigned length);
    static PassRefPtr<WebGLFloatArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    float* data() { return static_cast<float*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        float* storage = static_cast<float*>(m_baseAddress);
        storage[index] = static_cast<float>(value);
    }

    bool get(unsigned index, float& result) const
    {
        if (index >= m_size)
            return false;
        result = item(index);
        return true;
    }

    float get(unsigned index) const
    {
        return item(index);
    }

    float item(unsigned index) const
    {
        ASSERT(index < m_size);
        float* storage = static_cast<float*>(m_baseAddress);
        float result = storage[index];
        if (isnan(result)) {
            // Clamp NaN to 0
            result = 0;
        }
        return result;
    }

    void set(WebGLFloatArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLFloatArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLFloatArray_h
