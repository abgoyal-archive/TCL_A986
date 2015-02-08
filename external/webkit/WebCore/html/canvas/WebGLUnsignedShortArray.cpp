

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedShortArray.h"

namespace WebCore {

PassRefPtr<WebGLUnsignedShortArray> WebGLUnsignedShortArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(unsigned short));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLUnsignedShortArray> WebGLUnsignedShortArray::create(unsigned short* array, unsigned length)
{
    RefPtr<WebGLUnsignedShortArray> a = WebGLUnsignedShortArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLUnsignedShortArray> WebGLUnsignedShortArray::create(PassRefPtr<WebGLArrayBuffer> buffer,
                                                                    int byteOffset,
                                                                    unsigned length)
{
    // Make sure the offset results in valid alignment.
    if ((byteOffset % sizeof(unsigned short)) != 0) {
        return NULL;
    }

    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(unsigned short))) > buffer->byteLength()) 
            return NULL;
    }

    return adoptRef(new WebGLUnsignedShortArray(buffer, byteOffset, length));
}

WebGLUnsignedShortArray::WebGLUnsignedShortArray(PassRefPtr<WebGLArrayBuffer> buffer,
                                                 int byteOffset,
                                                 unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLUnsignedShortArray::length() const {
    return m_size;
}

unsigned WebGLUnsignedShortArray::byteLength() const {
    return m_size * sizeof(unsigned short);
}

PassRefPtr<WebGLArray> WebGLUnsignedShortArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(unsigned short);
    unsigned limitByte = startByte + length * sizeof(unsigned short);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLUnsignedShortArray::set(WebGLUnsignedShortArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(unsigned short), ec);
}

}

#endif // ENABLE(3D_CANVAS)
