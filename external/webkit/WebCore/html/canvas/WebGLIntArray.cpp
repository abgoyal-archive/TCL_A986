

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLIntArray.h"

namespace WebCore {

PassRefPtr<WebGLIntArray> WebGLIntArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(int));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLIntArray> WebGLIntArray::create(int* array, unsigned length)
{
    RefPtr<WebGLIntArray> a = WebGLIntArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLIntArray> WebGLIntArray::create(PassRefPtr<WebGLArrayBuffer> buffer,
                                                int byteOffset,
                                                unsigned length)
{
    // Make sure the offset results in valid alignment.
    if ((byteOffset % sizeof(int)) != 0)
        return NULL;

    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(int))) > buffer->byteLength())
            return NULL;
    }

    return adoptRef(new WebGLIntArray(buffer, byteOffset, length));
}

WebGLIntArray::WebGLIntArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLIntArray::length() const {
    return m_size;
}

unsigned WebGLIntArray::byteLength() const {
    return m_size * sizeof(int);
}

PassRefPtr<WebGLArray> WebGLIntArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(int);
    unsigned limitByte = startByte + length * sizeof(int);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLIntArray::set(WebGLIntArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(int), ec);
}

}

#endif // ENABLE(3D_CANVAS)
