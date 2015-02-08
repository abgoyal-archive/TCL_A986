

#ifndef WebGLBuffer_h
#define WebGLBuffer_h

#include "CanvasObject.h"
#include "WebGLArrayBuffer.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLBuffer : public CanvasObject {
    public:
        virtual ~WebGLBuffer() { deleteObject(); }
        
        static PassRefPtr<WebGLBuffer> create(WebGLRenderingContext*);
        
        // For querying previously created objects via e.g. getFramebufferAttachmentParameter
        // FIXME: should consider canonicalizing these objects
        static PassRefPtr<WebGLBuffer> create(WebGLRenderingContext*, Platform3DObject);

        bool associateBufferData(unsigned long target, int size);
        bool associateBufferData(unsigned long target, WebGLArray* array);
        bool associateBufferSubData(unsigned long target, long offset, WebGLArray* array);
        
        unsigned byteLength(unsigned long target) const;
        const WebGLArrayBuffer* elementArrayBuffer() const { return m_elementArrayBuffer.get(); }
                        
        // Gets the cached max index for the given type. Returns -1 if
        // none has been set.
        long getCachedMaxIndex(unsigned long type);
        // Sets the cached max index for the given type.
        void setCachedMaxIndex(unsigned long type, long value);

    protected:
        WebGLBuffer(WebGLRenderingContext*);
        WebGLBuffer(WebGLRenderingContext*, Platform3DObject obj);
        
        virtual void _deleteObject(Platform3DObject o);
    
    private:
        RefPtr<WebGLArrayBuffer> m_elementArrayBuffer;
        unsigned m_elementArrayBufferByteLength;
        unsigned m_arrayBufferByteLength;

        // Optimization for index validation. For each type of index
        // (i.e., UNSIGNED_SHORT), cache the maximum index in the
        // entire buffer.
        // 
        // This is sufficient to eliminate a lot of work upon each
        // draw call as long as all bound array buffers are at least
        // that size.
        struct MaxIndexCacheEntry {
            unsigned long type;
            long maxIndex;
        };
        // OpenGL ES 2.0 only has two valid index types (UNSIGNED_BYTE
        // and UNSIGNED_SHORT), but might as well leave open the
        // possibility of adding others.
        MaxIndexCacheEntry m_maxIndexCache[4];
        unsigned m_nextAvailableCacheEntry;

        // Clears all of the cached max indices.
        void clearCachedMaxIndices();
    };
    
} // namespace WebCore

#endif // WebGLBuffer_h
