

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLByteArray.h"

namespace WebCore {

PassRefPtr<WebGLByteArray> WebGLByteArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(signed char));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLByteArray> WebGLByteArray::create(signed char* array, unsigned length)
{
    RefPtr<WebGLByteArray> a = WebGLByteArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLByteArray> WebGLByteArray::create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
{
    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(signed char))) > buffer->byteLength())
            return NULL;
    }

    return adoptRef(new WebGLByteArray(buffer, byteOffset, length));
}

WebGLByteArray::WebGLByteArray(PassRefPtr<WebGLArrayBuffer> buffer, int offset, unsigned length)
    : WebGLArray(buffer, offset)
    , m_size(length)
{
}

unsigned WebGLByteArray::length() const {
    return m_size;
}

unsigned WebGLByteArray::byteLength() const {
    return m_size * sizeof(signed char);
}

PassRefPtr<WebGLArray> WebGLByteArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(signed char);
    unsigned limitByte = startByte + length * sizeof(signed char);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLByteArray::set(WebGLByteArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(signed char), ec);
}

}

#endif // ENABLE(3D_CANVAS)
