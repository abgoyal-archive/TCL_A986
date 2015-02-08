

#ifndef WebGLArrayBuffer_h
#define WebGLArrayBuffer_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLArrayBuffer : public RefCounted<WebGLArrayBuffer> {
  public:
    static PassRefPtr<WebGLArrayBuffer> create(unsigned sizeInBytes);
    static PassRefPtr<WebGLArrayBuffer> create(WebGLArrayBuffer*);

    void* data();
    const void* data() const;
    unsigned byteLength() const;

    ~WebGLArrayBuffer();

  private:
    WebGLArrayBuffer(unsigned sizeInBytes);
    unsigned m_sizeInBytes;
    void* m_data;
};

} // namespace WebCore

#endif // WebGLArrayBuffer_h
