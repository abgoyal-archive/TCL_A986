

#ifndef WebGLUnsignedShortArray_h
#define WebGLUnsignedShortArray_h

#include "WebGLArray.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLUnsignedShortArray : public WebGLArray {
  public:
    virtual bool isUnsignedShortArray() const { return true; }

    static PassRefPtr<WebGLUnsignedShortArray> create(unsigned length);
    static PassRefPtr<WebGLUnsignedShortArray> create(unsigned short* array, unsigned length);
    static PassRefPtr<WebGLUnsignedShortArray> create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length);

    unsigned short* data() { return static_cast<unsigned short*>(baseAddress()); }

    virtual unsigned length() const;
    virtual unsigned byteLength() const;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length);

    void set(unsigned index, double value)
    {
        if (index >= m_size)
            return;
        if (isnan(value)) // Clamp NaN to 0
            value = 0;
        if (value < std::numeric_limits<unsigned short>::min())
            value = std::numeric_limits<unsigned short>::min();
        else if (value > std::numeric_limits<unsigned short>::max())
            value = std::numeric_limits<unsigned short>::max();
        unsigned short* storage = static_cast<unsigned short*>(m_baseAddress);
        storage[index] = static_cast<unsigned short>(value);
    }

    bool get(unsigned index, unsigned short& result) const
    {
        if (index >= m_size)
            return false;
        unsigned short* storage = static_cast<unsigned short*>(m_baseAddress);
        result = storage[index];
        return true;
    }

    unsigned short get(unsigned index) const
    {
        return item(index);
    }

    unsigned short item(unsigned index) const
    {
        ASSERT(index < m_size);
        unsigned short* storage = static_cast<unsigned short*>(m_baseAddress);
        return storage[index];
    }

    void set(WebGLUnsignedShortArray* array, unsigned offset, ExceptionCode& ec);

  private:
    WebGLUnsignedShortArray(PassRefPtr<WebGLArrayBuffer> buffer,int byteOffset,unsigned length);
    unsigned m_size;
};

} // namespace WebCore

#endif // WebGLUnsignedShortArray_h
