

#ifndef WebGLRenderbuffer_h
#define WebGLRenderbuffer_h

#include "CanvasObject.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLRenderbuffer : public CanvasObject {
    public:
        virtual ~WebGLRenderbuffer() { deleteObject(); }
        
        static PassRefPtr<WebGLRenderbuffer> create(WebGLRenderingContext*);
        
        // For querying previously created objects via e.g. getFramebufferAttachmentParameter
        // FIXME: should consider canonicalizing these objects
        static PassRefPtr<WebGLRenderbuffer> create(WebGLRenderingContext*, Platform3DObject renderbuffer);

    protected:
        WebGLRenderbuffer(WebGLRenderingContext*);
        WebGLRenderbuffer(WebGLRenderingContext*, Platform3DObject);
        
        virtual void _deleteObject(Platform3DObject);
    };
    
} // namespace WebCore

#endif // WebGLRenderbuffer_h
