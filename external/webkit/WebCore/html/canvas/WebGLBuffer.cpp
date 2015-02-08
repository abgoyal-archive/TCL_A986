

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLBuffer.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLBuffer> WebGLBuffer::create(WebGLRenderingContext* ctx)
{
    return adoptRef(new WebGLBuffer(ctx));
}

PassRefPtr<WebGLBuffer> WebGLBuffer::create(WebGLRenderingContext* ctx, Platform3DObject obj)
{
    return adoptRef(new WebGLBuffer(ctx, obj));
}

WebGLBuffer::WebGLBuffer(WebGLRenderingContext* ctx)
    : CanvasObject(ctx)
    , m_elementArrayBufferByteLength(0)
    , m_arrayBufferByteLength(0)
    , m_nextAvailableCacheEntry(0)
{
    setObject(context()->graphicsContext3D()->createBuffer());
    clearCachedMaxIndices();
}

WebGLBuffer::WebGLBuffer(WebGLRenderingContext* ctx, Platform3DObject obj)
    : CanvasObject(ctx)
    , m_nextAvailableCacheEntry(0)
{
    setObject(obj, false);
    clearCachedMaxIndices();
}

void WebGLBuffer::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteBuffer(object);
}

bool WebGLBuffer::associateBufferData(unsigned long target, int size)
{
    if (target == GraphicsContext3D::ELEMENT_ARRAY_BUFFER) {
        m_elementArrayBufferByteLength = size;
        return true;
    }
    
    if (target == GraphicsContext3D::ARRAY_BUFFER) {
        m_arrayBufferByteLength = size;
        return true;
    }

    return false;
}

bool WebGLBuffer::associateBufferData(unsigned long target, WebGLArray* array)
{
    if (!array)
        return false;
        
    if (target == GraphicsContext3D::ELEMENT_ARRAY_BUFFER) {
        clearCachedMaxIndices();
        m_elementArrayBufferByteLength = array->byteLength();
        // We must always clone the incoming data because client-side
        // modifications without calling bufferData or bufferSubData
        // must never be able to change the validation results.
        m_elementArrayBuffer = WebGLArrayBuffer::create(array->buffer().get());
        return true;
    }
    
    if (target == GraphicsContext3D::ARRAY_BUFFER) {
        m_arrayBufferByteLength = array->byteLength();
        return true;
    }
    
    return false;
}

bool WebGLBuffer::associateBufferSubData(unsigned long target, long offset, WebGLArray* array)
{
    if (!array)
        return false;
        
    if (target == GraphicsContext3D::ELEMENT_ARRAY_BUFFER) {
        clearCachedMaxIndices();

        // We need to protect against integer overflow with these tests
        if (offset < 0)
            return false;
            
        unsigned long uoffset = static_cast<unsigned long>(offset);
        if (uoffset > m_elementArrayBufferByteLength || array->byteLength() > m_elementArrayBufferByteLength - uoffset)
            return false;
            
        memcpy(static_cast<unsigned char*>(m_elementArrayBuffer->data()) + offset, array->baseAddress(), array->byteLength());
        return true;
    }
    
    if (target == GraphicsContext3D::ARRAY_BUFFER)
        return array->byteLength() + offset <= m_arrayBufferByteLength;

    return false;
}

unsigned WebGLBuffer::byteLength(unsigned long target) const
{
    return (target == GraphicsContext3D::ARRAY_BUFFER) ? m_arrayBufferByteLength : m_elementArrayBufferByteLength;
}

long WebGLBuffer::getCachedMaxIndex(unsigned long type)
{
    size_t numEntries = sizeof(m_maxIndexCache) / sizeof(MaxIndexCacheEntry);
    for (size_t i = 0; i < numEntries; i++)
        if (m_maxIndexCache[i].type == type)
            return m_maxIndexCache[i].maxIndex;
    return -1;
}

void WebGLBuffer::setCachedMaxIndex(unsigned long type, long value)
{
    int numEntries = sizeof(m_maxIndexCache) / sizeof(MaxIndexCacheEntry);
    for (int i = 0; i < numEntries; i++)
        if (m_maxIndexCache[i].type == type) {
            m_maxIndexCache[i].maxIndex = value;
            return;
        }
    m_maxIndexCache[m_nextAvailableCacheEntry].type = type;
    m_maxIndexCache[m_nextAvailableCacheEntry].maxIndex = value;
    m_nextAvailableCacheEntry = (m_nextAvailableCacheEntry + 1) % numEntries;
}

void WebGLBuffer::clearCachedMaxIndices()
{
    memset(m_maxIndexCache, 0, sizeof(m_maxIndexCache));
}

}

#endif // ENABLE(3D_CANVAS)
