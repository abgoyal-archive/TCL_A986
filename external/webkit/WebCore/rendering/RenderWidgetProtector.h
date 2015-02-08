

#ifndef RenderWidgetProtector_h
#define RenderWidgetProtector_h

#include "RenderWidget.h"

namespace WebCore {

class RenderWidgetProtector : private Noncopyable {
public:
    RenderWidgetProtector(RenderWidget* object)
        : m_object(object)
        , m_arena(object->ref())
    {
    }

    ~RenderWidgetProtector()
    {
        m_object->deref(m_arena);
    }

private:
    RenderWidget* m_object;
    RenderArena* m_arena;
};

}

#endif // RenderWidgetProtector_h
