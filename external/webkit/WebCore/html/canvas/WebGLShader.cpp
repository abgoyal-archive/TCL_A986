

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLShader.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
PassRefPtr<WebGLShader> WebGLShader::create(WebGLRenderingContext* ctx, GraphicsContext3D::WebGLEnumType type)
{
    return adoptRef(new WebGLShader(ctx, type));
}

WebGLShader::WebGLShader(WebGLRenderingContext* ctx, GraphicsContext3D::WebGLEnumType type)
    : CanvasObject(ctx)
{
    setObject(context()->graphicsContext3D()->createShader(type));
}

void WebGLShader::_deleteObject(Platform3DObject object)
{
    context()->graphicsContext3D()->deleteShader(object);
}

}

#endif // ENABLE(3D_CANVAS)
