

#ifndef Generator_h
#define Generator_h

#include <wtf/RefCounted.h>

namespace WebCore {

class FloatRect;
class GraphicsContext;

class Generator : public RefCounted<Generator> {
public:
    virtual ~Generator() {};
    
    virtual void fill(GraphicsContext*, const FloatRect&) = 0;
    virtual void adjustParametersForTiledDrawing(IntSize& /* size */, FloatRect& /* srcRect */) { }
};

} //namespace

#endif
