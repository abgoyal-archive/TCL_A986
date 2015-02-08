

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLProgram.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLProgram> WebGLProgram::create(WebGLRenderingContext* ctx)
{
    return adoptRef(new WebGLProgram(ctx));
}

WebGLProgram::WebGLProgram(WebGLRenderingContext* ctx)
    : CanvasObject(ctx)
{
    setObject(context()->graphicsContext3D()->createProgram());
}

void WebGLProgram::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteProgram(object);
}

}

#endif // ENABLE(3D_CANVAS)
