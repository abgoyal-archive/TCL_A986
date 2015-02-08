

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedIntArray.h"

namespace WebCore {

PassRefPtr<WebGLUnsignedIntArray> WebGLUnsignedIntArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(unsigned int));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLUnsignedIntArray> WebGLUnsignedIntArray::create(unsigned int* array, unsigned length)
{
    RefPtr<WebGLUnsignedIntArray> a = WebGLUnsignedIntArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLUnsignedIntArray> WebGLUnsignedIntArray::create(PassRefPtr<WebGLArrayBuffer> buffer,
                                                                int byteOffset,
                                                                unsigned length)
{
    // Make sure the offset results in valid alignment.
    if ((byteOffset % sizeof(unsigned int)) != 0) {
        return NULL;
    }

    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(unsigned int))) > buffer->byteLength())
            return NULL;
    }

    return adoptRef(new WebGLUnsignedIntArray(buffer, byteOffset, length));
}

WebGLUnsignedIntArray::WebGLUnsignedIntArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLUnsignedIntArray::length() const {
    return m_size;
}

unsigned WebGLUnsignedIntArray::byteLength() const {
    return m_size * sizeof(unsigned int);
}

PassRefPtr<WebGLArray> WebGLUnsignedIntArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(unsigned int);
    unsigned limitByte = startByte + length * sizeof(unsigned int);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLUnsignedIntArray::set(WebGLUnsignedIntArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(unsigned int), ec);
}

}

#endif // ENABLE(3D_CANVAS)
