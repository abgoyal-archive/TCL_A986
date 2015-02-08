

#ifndef CanvasContextAttributes_h
#define CanvasContextAttributes_h

#include <wtf/RefCounted.h>

namespace WebCore {

// A base class for any attributes that are needed which would affect
// the creation of the Canvas's rendering context. Currently only the
// WebGLRenderingContext uses this mechanism.

class CanvasContextAttributes : public RefCounted<CanvasContextAttributes> {
  public:
    virtual ~CanvasContextAttributes();

  protected:
    CanvasContextAttributes();
};

} // namespace WebCore

#endif // CanvasContextAttributes_h
