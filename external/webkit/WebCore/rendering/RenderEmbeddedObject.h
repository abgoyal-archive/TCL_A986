

#ifndef RenderEmbeddedObject_h
#define RenderEmbeddedObject_h

#include "RenderPartObject.h"

namespace WebCore {

// Renderer for embeds and objects.
class RenderEmbeddedObject : public RenderPartObject {
public:
    RenderEmbeddedObject(Element*);
    virtual ~RenderEmbeddedObject();

    void updateWidget(bool onlyCreateNonNetscapePlugins);

#if USE(ACCELERATED_COMPOSITING)
    virtual bool allowsAcceleratedCompositing() const;
#endif

private:
    virtual const char* renderName() const { return "RenderEmbeddedObject"; }
    virtual bool isEmbeddedObject() const { return true; }

#if USE(ACCELERATED_COMPOSITING)
    virtual bool requiresLayer() const;
#endif

    virtual void layout();
};

inline RenderEmbeddedObject* toRenderEmbeddedObject(RenderObject* object)
{
    ASSERT(!object || !strcmp(object->renderName(), "RenderEmbeddedObject"));
    return static_cast<RenderEmbeddedObject*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderEmbeddedObject(const RenderEmbeddedObject*);

} // namespace WebCore

#endif // RenderEmbeddedObject_h
