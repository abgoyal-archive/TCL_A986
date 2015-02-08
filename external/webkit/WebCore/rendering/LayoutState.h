

#ifndef LayoutState_h
#define LayoutState_h

#include "IntRect.h"
#include "IntSize.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

class RenderArena;
class RenderBox;
class RenderObject;

class LayoutState : public Noncopyable {
public:
    LayoutState()
        : m_clipped(false)
        , m_next(0)
#ifndef NDEBUG
        , m_renderer(0)
#endif
    {
    }

    LayoutState(LayoutState*, RenderBox*, const IntSize& offset);
    LayoutState(RenderObject*);

    void destroy(RenderArena*);

    // Overloaded new operator.
    void* operator new(size_t, RenderArena*) throw();

    // Overridden to prevent the normal delete from being called.
    void operator delete(void*, size_t);

private:
    // The normal operator new is disallowed.
    void* operator new(size_t) throw();

public:
    bool m_clipped;
    IntRect m_clipRect;
    IntSize m_offset;       // x/y offset from container.
    IntSize m_layoutDelta;  // Transient offset from the final position of the object
                            // used to ensure that repaints happen in the correct place.
                            // This is a total delta accumulated from the root.
    LayoutState* m_next;
#ifndef NDEBUG
    RenderObject* m_renderer;
#endif
};

} // namespace WebCore

#endif // LayoutState_h
