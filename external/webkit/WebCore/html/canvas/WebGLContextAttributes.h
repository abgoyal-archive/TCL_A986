

#ifndef WebGLContextAttributes_h
#define WebGLContextAttributes_h

#include "CanvasContextAttributes.h"
#include "GraphicsContext3D.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

class WebGLContextAttributes : public CanvasContextAttributes {
  public:
    virtual ~WebGLContextAttributes();

    // Create a new attributes object
    static PassRefPtr<WebGLContextAttributes> create();

    // Create a new attributes object initialized with preexisting attributes
    static PassRefPtr<WebGLContextAttributes> create(GraphicsContext3D::Attributes attributes);

    // Whether or not the drawing buffer has an alpha channel; default=true
    bool alpha() const;
    void setAlpha(bool alpha);

    // Whether or not the drawing buffer has a depth buffer; default=true
    bool depth() const;
    void setDepth(bool depth);

    // Whether or not the drawing buffer has a stencil buffer; default=true
    bool stencil() const;
    void setStencil(bool stencil);

    // Whether or not the drawing buffer is antialiased; default=true
    bool antialias() const;
    void setAntialias(bool antialias);

    // Whether or not to treat the values in the drawing buffer as
    // though their alpha channel has already been multiplied into the
    // color channels; default=true
    bool premultipliedAlpha() const;
    void setPremultipliedAlpha(bool premultipliedAlpha);

    // Fetches a copy of the attributes stored in this object in a
    // form that can be used to initialize a GraphicsContext3D.
    GraphicsContext3D::Attributes attributes() const;

  protected:
    WebGLContextAttributes();
    WebGLContextAttributes(GraphicsContext3D::Attributes attributes);

  private:
    GraphicsContext3D::Attributes m_attrs;
};

} // namespace WebCore

#endif // WebGLContextAttributes_h
