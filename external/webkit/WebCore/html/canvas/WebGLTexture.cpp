

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLTexture.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLTexture> WebGLTexture::create(WebGLRenderingContext* ctx)
{
    return adoptRef(new WebGLTexture(ctx));
}

PassRefPtr<WebGLTexture> WebGLTexture::create(WebGLRenderingContext* ctx, Platform3DObject obj)
{
    return adoptRef(new WebGLTexture(ctx, obj));
}

WebGLTexture::WebGLTexture(WebGLRenderingContext* ctx)
    : CanvasObject(ctx)
    , cubeMapRWrapModeInitialized(false)
{
    setObject(context()->graphicsContext3D()->createTexture());
}

WebGLTexture::WebGLTexture(WebGLRenderingContext* ctx, Platform3DObject obj)
    : CanvasObject(ctx)
    , cubeMapRWrapModeInitialized(false)
{
    setObject(obj, false);
}

void WebGLTexture::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteTexture(object);
}

}

#endif // ENABLE(3D_CANVAS)
