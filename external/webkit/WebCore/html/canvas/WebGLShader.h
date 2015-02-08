

#ifndef WebGLShader_h
#define WebGLShader_h

#include "CanvasObject.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {
    
    class WebGLShader : public CanvasObject {
    public:
        virtual ~WebGLShader() { deleteObject(); }
        
        static PassRefPtr<WebGLShader> create(WebGLRenderingContext*, GraphicsContext3D::WebGLEnumType);
        
    private:
        WebGLShader(WebGLRenderingContext*, GraphicsContext3D::WebGLEnumType);

        virtual void _deleteObject(Platform3DObject);
    };
    
} // namespace WebCore

#endif // WebGLShader_h
