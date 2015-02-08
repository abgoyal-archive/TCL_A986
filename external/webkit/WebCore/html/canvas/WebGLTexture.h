

#ifndef WebGLTexture_h
#define WebGLTexture_h

#include "CanvasObject.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLTexture : public CanvasObject {
    public:
        virtual ~WebGLTexture() { deleteObject(); }
        
        static PassRefPtr<WebGLTexture> create(WebGLRenderingContext*);
    
        // For querying previously created objects via e.g. getFramebufferAttachmentParameter
        // FIXME: should consider canonicalizing these objects
        static PassRefPtr<WebGLTexture> create(WebGLRenderingContext*, Platform3DObject);

        bool isCubeMapRWrapModeInitialized() {
            return cubeMapRWrapModeInitialized;
        }

        void setCubeMapRWrapModeInitialized(bool initialized) {
            cubeMapRWrapModeInitialized = initialized;
        }

    protected:
        WebGLTexture(WebGLRenderingContext*);
        WebGLTexture(WebGLRenderingContext*, Platform3DObject);

        virtual void _deleteObject(Platform3DObject);

    private:
        bool cubeMapRWrapModeInitialized;
    };
    
} // namespace WebCore

#endif // WebGLTexture_h
