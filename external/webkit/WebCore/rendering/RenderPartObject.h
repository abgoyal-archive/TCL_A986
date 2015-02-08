

#ifndef RenderPartObject_h
#define RenderPartObject_h

#include "RenderPart.h"

namespace WebCore {

// Renderer for iframes. Is subclassed in RenderEmbeddedObject for object and embed.
class RenderPartObject : public RenderPart {
public:
    RenderPartObject(Element*);

#ifdef FLATTEN_IFRAME
protected:
    virtual void calcWidth();
    virtual void calcHeight();
#endif

private:
    virtual const char* renderName() const { return "RenderPartObject"; }

    virtual void layout();

    virtual void viewCleared();
};

inline RenderPartObject* toRenderPartObject(RenderObject* object)
{
    ASSERT(!object || !strcmp(object->renderName(), "RenderPartObject"));
    return static_cast<RenderPartObject*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderPartObject(const RenderPartObject*);

} // namespace WebCore

#endif // RenderPartObject_h
