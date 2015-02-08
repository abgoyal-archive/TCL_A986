

#ifndef RenderHTMLCanvas_h
#define RenderHTMLCanvas_h

#include "RenderReplaced.h"

namespace WebCore {

class HTMLCanvasElement;

class RenderHTMLCanvas : public RenderReplaced {
public:
    RenderHTMLCanvas(HTMLCanvasElement*);

    virtual bool isCanvas() const { return true; }
    virtual bool requiresLayer() const;

    void canvasSizeChanged();
    
private:
    virtual const char* renderName() const { return "RenderHTMLCanvas"; }
    virtual void paintReplaced(PaintInfo&, int tx, int ty);
    virtual void intrinsicSizeChanged() { canvasSizeChanged(); }
};

inline RenderHTMLCanvas* toRenderHTMLCanvas(RenderObject* object)
{
    ASSERT(!object || !strcmp(object->renderName(), "RenderHTMLCanvas"));
    return static_cast<RenderHTMLCanvas*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderHTMLCanvas(const RenderHTMLCanvas*);

} // namespace WebCore

#endif // RenderHTMLCanvas_h
