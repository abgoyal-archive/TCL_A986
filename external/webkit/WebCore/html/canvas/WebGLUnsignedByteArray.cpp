

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedByteArray.h"

namespace WebCore {

PassRefPtr<WebGLUnsignedByteArray> WebGLUnsignedByteArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(unsigned char));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLUnsignedByteArray> WebGLUnsignedByteArray::create(unsigned char* array, unsigned length)
{
    RefPtr<WebGLUnsignedByteArray> a = WebGLUnsignedByteArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLUnsignedByteArray> WebGLUnsignedByteArray::create(PassRefPtr<WebGLArrayBuffer> buffer,
                                                                  int byteOffset,
                                                                  unsigned length)
{
    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(unsigned char))) > buffer->byteLength())
            return NULL;
    }

    return adoptRef(new WebGLUnsignedByteArray(buffer, byteOffset, length));
}

WebGLUnsignedByteArray::WebGLUnsignedByteArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLUnsignedByteArray::length() const {
    return m_size;
}

unsigned WebGLUnsignedByteArray::byteLength() const {
    return m_size * sizeof(unsigned char);
}

PassRefPtr<WebGLArray> WebGLUnsignedByteArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(unsigned char);
    unsigned limitByte = startByte + length * sizeof(unsigned char);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLUnsignedByteArray::set(WebGLUnsignedByteArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(unsigned char), ec);
}

}

#endif // ENABLE(3D_CANVAS)
