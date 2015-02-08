

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLFloatArray.h"

namespace WebCore {

PassRefPtr<WebGLFloatArray> WebGLFloatArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(float));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLFloatArray> WebGLFloatArray::create(float* array, unsigned length)
{
    RefPtr<WebGLFloatArray> a = WebGLFloatArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLFloatArray> WebGLFloatArray::create(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
{
    // Make sure the offset results in valid alignment.
    if ((byteOffset % sizeof(float)) != 0)
        return NULL;

    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(float))) > buffer->byteLength())
            return NULL;
    }
    return adoptRef(new WebGLFloatArray(buffer, byteOffset, length));
}

WebGLFloatArray::WebGLFloatArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLFloatArray::length() const {
    return m_size;
}

unsigned WebGLFloatArray::byteLength() const {
    return m_size * sizeof(float);
}

PassRefPtr<WebGLArray> WebGLFloatArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(float);
    unsigned limitByte = startByte + length * sizeof(float);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLFloatArray::set(WebGLFloatArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(float), ec);
}

}

#endif // ENABLE(3D_CANVAS)
