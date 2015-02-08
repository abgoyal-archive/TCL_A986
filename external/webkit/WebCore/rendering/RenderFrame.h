

#ifndef RenderFrame_h
#define RenderFrame_h

#include "RenderPart.h"
#include "RenderFrameSet.h"

namespace WebCore {

class HTMLFrameElement;

class RenderFrame : public RenderPart {
public:
    RenderFrame(HTMLFrameElement*);

    FrameEdgeInfo edgeInfo() const;
    void layoutWithFlattening(bool fixedWidth, bool fixedHeight);

private:
    virtual const char* renderName() const { return "RenderFrame"; }
    virtual bool isFrame() const { return true; }

#ifdef FLATTEN_FRAMESET
    virtual void layout();
#endif
    virtual void viewCleared();
};

inline RenderFrame* toRenderFrame(RenderObject* object)
{
    ASSERT(!object || object->isFrame());
    return static_cast<RenderFrame*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderFrame(const RenderFrame*);

} // namespace WebCore

#endif // RenderFrame_h
