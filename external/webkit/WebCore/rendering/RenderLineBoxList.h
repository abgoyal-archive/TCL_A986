


#ifndef RenderLineBoxList_h
#define RenderLineBoxList_h

#include "RenderBox.h"

namespace WebCore {

class RenderLineBoxList {
public:
    RenderLineBoxList()
        : m_firstLineBox(0)
        , m_lastLineBox(0)
    {
    }

#ifndef NDEBUG
    ~RenderLineBoxList();
#endif

    InlineFlowBox* firstLineBox() const { return m_firstLineBox; }
    InlineFlowBox* lastLineBox() const { return m_lastLineBox; }

    void checkConsistency() const;

    void appendLineBox(InlineFlowBox*);

    void deleteLineBoxTree(RenderArena*);
    void deleteLineBoxes(RenderArena*);

    void extractLineBox(InlineFlowBox*);
    void attachLineBox(InlineFlowBox*);
    void removeLineBox(InlineFlowBox*);
    
    void dirtyLineBoxes();
    void dirtyLinesFromChangedChild(RenderObject* parent, RenderObject* child);

    void paint(RenderBoxModelObject*, RenderObject::PaintInfo&, int x, int y) const;
    bool hitTest(RenderBoxModelObject*, const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction) const;
    
private:
    // For block flows, each box represents the root inline box for a line in the
    // paragraph.
    // For inline flows, each box represents a portion of that inline.
    InlineFlowBox* m_firstLineBox;
    InlineFlowBox* m_lastLineBox;
};


#ifdef NDEBUG
inline void RenderLineBoxList::checkConsistency() const
{
}
#endif

} // namespace WebCore

#endif // RenderFlow_h
