

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLShortArray.h"

namespace WebCore {

PassRefPtr<WebGLShortArray> WebGLShortArray::create(unsigned length)
{
    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(length * sizeof(short));
    return create(buffer, 0, length);
}

PassRefPtr<WebGLShortArray> WebGLShortArray::create(short* array, unsigned length)
{
    RefPtr<WebGLShortArray> a = WebGLShortArray::create(length);
    for (unsigned i = 0; i < length; ++i)
        a->set(i, array[i]);
    return a;
}

PassRefPtr<WebGLShortArray> WebGLShortArray::create(PassRefPtr<WebGLArrayBuffer> buffer,
                                                    int byteOffset,
                                                    unsigned length)
{
    // Make sure the offset results in valid alignment.
    if ((byteOffset % sizeof(short)) != 0)
        return NULL;

    if (buffer) {
        // Check to make sure we are talking about a valid region of
        // the given WebGLArrayBuffer's storage.
        if ((byteOffset + (length * sizeof(short))) > buffer->byteLength())
            return NULL;
    }

    return adoptRef(new WebGLShortArray(buffer, byteOffset, length));
}

WebGLShortArray::WebGLShortArray(PassRefPtr<WebGLArrayBuffer> buffer, int byteOffset, unsigned length)
        : WebGLArray(buffer, byteOffset)
        , m_size(length)
{
}

unsigned WebGLShortArray::length() const {
    return m_size;
}

unsigned WebGLShortArray::byteLength() const {
    return m_size * sizeof(short);
}

PassRefPtr<WebGLArray> WebGLShortArray::slice(unsigned offset, unsigned length) {
    // Check to make sure the specified region is within the bounds of
    // the WebGLArrayBuffer.
    unsigned startByte = m_byteOffset + offset * sizeof(short);
    unsigned limitByte = startByte + length * sizeof(short);
    unsigned bufferLength = buffer()->byteLength();
    if (startByte >= bufferLength || limitByte > bufferLength)
        return 0;
    return create(buffer(), startByte, length);
}

void WebGLShortArray::set(WebGLShortArray* array, unsigned offset, ExceptionCode& ec) {
    setImpl(array, offset * sizeof(short), ec);
}

}

#endif // ENABLE(3D_CANVAS)
