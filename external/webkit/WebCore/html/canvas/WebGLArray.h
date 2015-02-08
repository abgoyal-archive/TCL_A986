

#ifndef WebGLArray_h
#define WebGLArray_h

#include "ExceptionCode.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include "WebGLArrayBuffer.h"

namespace WebCore {

class WebGLArray : public RefCounted<WebGLArray> {
  public:
    virtual bool isByteArray() const { return false; }
    virtual bool isUnsignedByteArray() const { return false; }
    virtual bool isShortArray() const { return false; }
    virtual bool isUnsignedShortArray() const { return false; }
    virtual bool isIntArray() const { return false; }
    virtual bool isUnsignedIntArray() const { return false; }
    virtual bool isFloatArray() const { return false; }

    PassRefPtr<WebGLArrayBuffer> buffer() {
        return m_buffer;
    }

    void* baseAddress() {
        return m_baseAddress;
    }

    unsigned byteOffset() const {
        return m_byteOffset;
    }

    virtual unsigned length() const = 0;
    virtual unsigned byteLength() const = 0;
    virtual PassRefPtr<WebGLArray> slice(unsigned offset, unsigned length) = 0;

    virtual ~WebGLArray();

  protected:
    WebGLArray(PassRefPtr<WebGLArrayBuffer> buffer, unsigned byteOffset);

    void setImpl(WebGLArray* array, unsigned byteOffset, ExceptionCode& ec);

    // This is the address of the WebGLArrayBuffer's storage, plus the byte offset.
    void* m_baseAddress;

    unsigned m_byteOffset;

  private:
    RefPtr<WebGLArrayBuffer> m_buffer;
};

} // namespace WebCore

#endif // WebGLArray_h
