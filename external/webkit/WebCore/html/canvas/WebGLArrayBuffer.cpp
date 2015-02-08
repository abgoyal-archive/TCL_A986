

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"

#include <wtf/RefPtr.h>

namespace WebCore {

PassRefPtr<WebGLArrayBuffer> WebGLArrayBuffer::create(unsigned sizeInBytes)
{
    return adoptRef(new WebGLArrayBuffer(sizeInBytes));
}

PassRefPtr<WebGLArrayBuffer> WebGLArrayBuffer::create(WebGLArrayBuffer* other)
{
    RefPtr<WebGLArrayBuffer> buffer = adoptRef(new WebGLArrayBuffer(other->byteLength()));
    memcpy(buffer->data(), other->data(), other->byteLength());
    return buffer.release();
}

WebGLArrayBuffer::WebGLArrayBuffer(unsigned sizeInBytes) {
    m_sizeInBytes = sizeInBytes;
    m_data = WTF::fastZeroedMalloc(sizeInBytes);
}

void* WebGLArrayBuffer::data() {
    return m_data;
}

const void* WebGLArrayBuffer::data() const {
    return m_data;
}

unsigned WebGLArrayBuffer::byteLength() const {
    return m_sizeInBytes;
}

WebGLArrayBuffer::~WebGLArrayBuffer() {
    WTF::fastFree(m_data);
}

}

#endif // ENABLE(3D_CANVAS)
