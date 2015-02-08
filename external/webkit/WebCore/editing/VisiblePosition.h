

#ifndef VisiblePosition_h
#define VisiblePosition_h

#include "Node.h"
#include "Position.h"
#include "TextDirection.h"

namespace WebCore {

// VisiblePosition default affinity is downstream because
// the callers do not really care (they just want the
// deep position without regard to line position), and this
// is cheaper than UPSTREAM
#define VP_DEFAULT_AFFINITY DOWNSTREAM

// Callers who do not know where on the line the position is,
// but would like UPSTREAM if at a line break or DOWNSTREAM
// otherwise, need a clear way to specify that.  The
// constructors auto-correct UPSTREAM to DOWNSTREAM if the
// position is not at a line break.
#define VP_UPSTREAM_IF_POSSIBLE UPSTREAM

class InlineBox;

enum StayInEditableContent { MayLeaveEditableContent, MustStayInEditableContent };

class VisiblePosition {
public:
    // NOTE: UPSTREAM affinity will be used only if pos is at end of a wrapped line,
    // otherwise it will be converted to DOWNSTREAM
    VisiblePosition() : m_affinity(VP_DEFAULT_AFFINITY) { }
    VisiblePosition(Node*, int offset, EAffinity);
    VisiblePosition(const Position&, EAffinity = VP_DEFAULT_AFFINITY);

    void clear() { m_deepPosition.clear(); }

    bool isNull() const { return m_deepPosition.isNull(); }
    bool isNotNull() const { return m_deepPosition.isNotNull(); }

    Position deepEquivalent() const { return m_deepPosition; }
    EAffinity affinity() const { ASSERT(m_affinity == UPSTREAM || m_affinity == DOWNSTREAM); return m_affinity; }
    void setAffinity(EAffinity affinity) { m_affinity = affinity; }

    // FIXME: Change the following functions' parameter from a boolean to StayInEditableContent.

    // next() and previous() will increment/decrement by a character cluster.
    VisiblePosition next(bool stayInEditableContent = false) const;
    VisiblePosition previous(bool stayInEditableContent = false) const;
    VisiblePosition honorEditableBoundaryAtOrBefore(const VisiblePosition&) const;
    VisiblePosition honorEditableBoundaryAtOrAfter(const VisiblePosition&) const;

    VisiblePosition left(bool stayInEditableContent = false) const;
    VisiblePosition right(bool stayInEditableContent = false) const;

    UChar32 characterAfter() const;
    UChar32 characterBefore() const { return previous().characterAfter(); }
    
    void debugPosition(const char* msg = "") const;
    
    Element* rootEditableElement() const { return m_deepPosition.isNotNull() ? m_deepPosition.node()->rootEditableElement() : 0; }
    
    void getInlineBoxAndOffset(InlineBox*& inlineBox, int& caretOffset) const
    {
        m_deepPosition.getInlineBoxAndOffset(m_affinity, inlineBox, caretOffset);
    }

    void getInlineBoxAndOffset(TextDirection primaryDirection, InlineBox*& inlineBox, int& caretOffset) const
    {
        m_deepPosition.getInlineBoxAndOffset(m_affinity, primaryDirection, inlineBox, caretOffset);
    }

    // Rect is local to the returned renderer
    IntRect localCaretRect(RenderObject*&) const;
    // Bounds of (possibly transformed) caret in absolute coords
    IntRect absoluteCaretBounds() const;
    // Abs x position of the caret ignoring transforms.
    // FIXME: navigation with transforms should be smarter.
    int xOffsetForVerticalNavigation() const;
    
#ifndef NDEBUG
    void formatForDebugger(char* buffer, unsigned length) const;
    void showTreeForThis() const;
#endif
    
private:
    void init(const Position&, EAffinity);
    Position canonicalPosition(const Position&);

    Position leftVisuallyDistinctCandidate() const;
    Position rightVisuallyDistinctCandidate() const;

    Position m_deepPosition;
    EAffinity m_affinity;
};

// FIXME: This shouldn't ignore affinity.
inline bool operator==(const VisiblePosition& a, const VisiblePosition& b)
{
    return a.deepEquivalent() == b.deepEquivalent();
}
 
inline bool operator!=(const VisiblePosition& a, const VisiblePosition& b)
{
    return !(a == b);
}

PassRefPtr<Range> makeRange(const VisiblePosition&, const VisiblePosition&);
bool setStart(Range*, const VisiblePosition&);
bool setEnd(Range*, const VisiblePosition&);
VisiblePosition startVisiblePosition(const Range*, EAffinity);
VisiblePosition endVisiblePosition(const Range*, EAffinity);

Node *enclosingBlockFlowElement(const VisiblePosition&);

bool isFirstVisiblePositionInNode(const VisiblePosition&, const Node*);
bool isLastVisiblePositionInNode(const VisiblePosition&, const Node*);

} // namespace WebCore

#ifndef NDEBUG
// Outside the WebCore namespace for ease of invocation from gdb.
void showTree(const WebCore::VisiblePosition*);
void showTree(const WebCore::VisiblePosition&);
#endif

#endif // VisiblePosition_h
