

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArray.h"
#include "WebGLArrayBuffer.h"

namespace WebCore {

WebGLArray::WebGLArray(PassRefPtr<WebGLArrayBuffer> buffer,
                       unsigned byteOffset)
        : m_byteOffset(byteOffset)
        , m_buffer(buffer)
{
    m_baseAddress = m_buffer ? (static_cast<char*>(m_buffer->data()) + m_byteOffset) : 0;
}

WebGLArray::~WebGLArray()
{
}

void WebGLArray::setImpl(WebGLArray* array, unsigned byteOffset, ExceptionCode& ec)
{
    if (byteOffset + array->byteLength() > byteLength()) {
        ec = INDEX_SIZE_ERR;
        return;
    }

    char* base = static_cast<char*>(baseAddress());
    memcpy(base + byteOffset, array->baseAddress(), array->byteLength());
}

}

#endif // ENABLE(3D_CANVAS)
