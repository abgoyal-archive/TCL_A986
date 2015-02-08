

#ifndef WebGLFramebuffer_h
#define WebGLFramebuffer_h

#include "CanvasObject.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLFramebuffer : public CanvasObject {
    public:
        virtual ~WebGLFramebuffer() { deleteObject(); }
        
        static PassRefPtr<WebGLFramebuffer> create(WebGLRenderingContext*);
        
    protected:
        WebGLFramebuffer(WebGLRenderingContext*);
        
        virtual void _deleteObject(Platform3DObject);
    };
    
} // namespace WebCore

#endif // WebGLFramebuffer_h
