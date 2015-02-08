

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLContextAttributes.h"

namespace WebCore {

PassRefPtr<WebGLContextAttributes> WebGLContextAttributes::create()
{
    return adoptRef(new WebGLContextAttributes());
}

PassRefPtr<WebGLContextAttributes> WebGLContextAttributes::create(GraphicsContext3D::Attributes attributes)
{
    return adoptRef(new WebGLContextAttributes(attributes));
}

WebGLContextAttributes::WebGLContextAttributes()
    : CanvasContextAttributes()
{
}

WebGLContextAttributes::WebGLContextAttributes(GraphicsContext3D::Attributes attributes)
    : CanvasContextAttributes()
    , m_attrs(attributes)
{
}

WebGLContextAttributes::~WebGLContextAttributes()
{
}

bool WebGLContextAttributes::alpha() const
{
    return m_attrs.alpha;
}

void WebGLContextAttributes::setAlpha(bool alpha)
{
    m_attrs.alpha = alpha;
}

bool WebGLContextAttributes::depth() const
{
    return m_attrs.depth;
}

void WebGLContextAttributes::setDepth(bool depth)
{
    m_attrs.depth = depth;
}

bool WebGLContextAttributes::stencil() const
{
    return m_attrs.stencil;
}

void WebGLContextAttributes::setStencil(bool stencil)
{
    m_attrs.stencil = stencil;
}

bool WebGLContextAttributes::antialias() const
{
    return m_attrs.antialias;
}

void WebGLContextAttributes::setAntialias(bool antialias)
{
    m_attrs.antialias = antialias;
}

bool WebGLContextAttributes::premultipliedAlpha() const
{
    return m_attrs.premultipliedAlpha;
}

void WebGLContextAttributes::setPremultipliedAlpha(bool premultipliedAlpha)
{
    m_attrs.premultipliedAlpha = premultipliedAlpha;
}

GraphicsContext3D::Attributes WebGLContextAttributes::attributes() const
{
    return m_attrs;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
