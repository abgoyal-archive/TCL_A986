

#ifndef WebGLProgram_h
#define WebGLProgram_h

#include "CanvasObject.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLProgram : public CanvasObject {
    public:
        virtual ~WebGLProgram() { deleteObject(); }
        
        static PassRefPtr<WebGLProgram> create(WebGLRenderingContext*);
        
    protected:
        WebGLProgram(WebGLRenderingContext*);
        
        virtual void _deleteObject(Platform3DObject);
    };
    
} // namespace WebCore

#endif // WebGLProgram_h
