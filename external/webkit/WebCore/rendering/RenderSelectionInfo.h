

#ifndef SelectionInfo_h
#define SelectionInfo_h

#include "IntRect.h"
#include "RenderBox.h"

namespace WebCore {

class RenderSelectionInfoBase : public Noncopyable {
public:
    RenderSelectionInfoBase()
        : m_object(0)
        , m_repaintContainer(0)
        , m_state(RenderObject::SelectionNone)
    {
    }

    RenderSelectionInfoBase(RenderObject* o)
        : m_object(o)
        , m_repaintContainer(o->containerForRepaint())
        , m_state(o->selectionState())
    {
    }
    
    RenderObject* object() const { return m_object; }
    RenderBoxModelObject* repaintContainer() const { return m_repaintContainer; }
    RenderObject::SelectionState state() const { return m_state; }

protected:
    RenderObject* m_object;
    RenderBoxModelObject* m_repaintContainer;
    RenderObject::SelectionState m_state;
};

// This struct is used when the selection changes to cache the old and new state of the selection for each RenderObject.
class RenderSelectionInfo : public RenderSelectionInfoBase {
public:
    RenderSelectionInfo(RenderObject* o, bool clipToVisibleContent)
        : RenderSelectionInfoBase(o)
        , m_rect(o->needsLayout() ? IntRect() : o->selectionRectForRepaint(m_repaintContainer, clipToVisibleContent))
    {
    }
    
    void repaint()
    {
        m_object->repaintUsingContainer(m_repaintContainer, m_rect);
    }

    IntRect rect() const { return m_rect; }

private:
    IntRect m_rect; // relative to repaint container
};


// This struct is used when the selection changes to cache the old and new state of the selection for each RenderBlock.
class RenderBlockSelectionInfo : public RenderSelectionInfoBase {
public:
    RenderBlockSelectionInfo(RenderBlock* b)
        : RenderSelectionInfoBase(b)
        , m_rects(b->needsLayout() ? GapRects() : block()->selectionGapRectsForRepaint(m_repaintContainer))
    { 
    }

    void repaint()
    {
        m_object->repaintUsingContainer(m_repaintContainer, m_rects);
    }
    
    RenderBlock* block() const { return toRenderBlock(m_object); }
    GapRects rects() const { return m_rects; }

private:
    GapRects m_rects; // relative to repaint container
};

} // namespace WebCore


#endif // SelectionInfo_h
