

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLFramebuffer.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLFramebuffer> WebGLFramebuffer::create(WebGLRenderingContext* ctx)
{
    return adoptRef(new WebGLFramebuffer(ctx));
}

WebGLFramebuffer::WebGLFramebuffer(WebGLRenderingContext* ctx)
    : CanvasObject(ctx)
{
    setObject(context()->graphicsContext3D()->createFramebuffer());
}

void WebGLFramebuffer::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteFramebuffer(object);
}

}

#endif // ENABLE(3D_CANVAS)
