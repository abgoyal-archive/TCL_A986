

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLRenderbuffer.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLRenderbuffer> WebGLRenderbuffer::create(WebGLRenderingContext* ctx)
{
    return adoptRef(new WebGLRenderbuffer(ctx));
}

PassRefPtr<WebGLRenderbuffer> WebGLRenderbuffer::create(WebGLRenderingContext* ctx, Platform3DObject obj)
{
    return adoptRef(new WebGLRenderbuffer(ctx, obj));
}

WebGLRenderbuffer::WebGLRenderbuffer(WebGLRenderingContext* ctx)
    : CanvasObject(ctx)
{
    setObject(context()->graphicsContext3D()->createRenderbuffer());
}

WebGLRenderbuffer::WebGLRenderbuffer(WebGLRenderingContext* ctx, Platform3DObject obj)
    : CanvasObject(ctx)
{
    setObject(obj, false);
}

void WebGLRenderbuffer::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteRenderbuffer(object);
}

}

#endif // ENABLE(3D_CANVAS)
