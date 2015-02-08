

#ifndef VisibleSelection_h
#define VisibleSelection_h

#include "TextGranularity.h"
#include "VisiblePosition.h"

namespace WebCore {

class Position;

const EAffinity SEL_DEFAULT_AFFINITY = DOWNSTREAM;

class VisibleSelection {
public:
    enum SelectionType { NoSelection, CaretSelection, RangeSelection };

    VisibleSelection();

    VisibleSelection(const Position&, EAffinity);
    VisibleSelection(const Position&, const Position&, EAffinity = SEL_DEFAULT_AFFINITY);

    VisibleSelection(const Range*, EAffinity = SEL_DEFAULT_AFFINITY);
    
    VisibleSelection(const VisiblePosition&);
    VisibleSelection(const VisiblePosition&, const VisiblePosition&);

    static VisibleSelection selectionFromContentsOfNode(Node*);

    SelectionType selectionType() const { return m_selectionType; }

    void setAffinity(EAffinity affinity) { m_affinity = affinity; }
    EAffinity affinity() const { return m_affinity; }

    void setBase(const Position&);
    void setBase(const VisiblePosition&);
    void setExtent(const Position&);
    void setExtent(const VisiblePosition&);
    
    Position base() const { return m_base; }
    Position extent() const { return m_extent; }
    Position start() const { return m_start; }
    Position end() const { return m_end; }
    
    VisiblePosition visibleStart() const { return VisiblePosition(m_start, isRange() ? DOWNSTREAM : affinity()); }
    VisiblePosition visibleEnd() const { return VisiblePosition(m_end, isRange() ? UPSTREAM : affinity()); }

    bool isNone() const { return selectionType() == NoSelection; }
    bool isCaret() const { return selectionType() == CaretSelection; }
    bool isRange() const { return selectionType() == RangeSelection; }
    bool isCaretOrRange() const { return selectionType() != NoSelection; }

    bool isBaseFirst() const { return m_baseIsFirst; }

    bool isAll(StayInEditableContent) const;

    void appendTrailingWhitespace();

    bool expandUsingGranularity(TextGranularity granularity);
    TextGranularity granularity() const { return m_granularity; }

    // We don't yet support multi-range selections, so we only ever have one range to return.
    PassRefPtr<Range> firstRange() const;

    // FIXME: Most callers probably don't want this function, but are using it
    // for historical reasons.  toNormalizedRange contracts the range around
    // text, and moves the caret upstream before returning the range.
    PassRefPtr<Range> toNormalizedRange() const;
    
    Element* rootEditableElement() const;
    bool isContentEditable() const;
    bool isContentRichlyEditable() const;
    Node* shadowTreeRootNode() const;
    
    void debugPosition() const;

#ifndef NDEBUG
    void formatForDebugger(char* buffer, unsigned length) const;
    void showTreeForThis() const;
#endif

    void setWithoutValidation(const Position&, const Position&);

private:
    void validate();

    // Support methods for validate()
    void setBaseAndExtentToDeepEquivalents();
    void setStartAndEndFromBaseAndExtentRespectingGranularity();
    void adjustSelectionToAvoidCrossingEditingBoundaries();
    void updateSelectionType();

    // We need to store these as Positions because VisibleSelection is
    // used to store values in editing commands for use when
    // undoing the command. We need to be able to create a selection that, while currently
    // invalid, will be valid once the changes are undone.
    
    Position m_base;   // Where the first click happened
    Position m_extent; // Where the end click happened
    Position m_start;  // Leftmost position when expanded to respect granularity
    Position m_end;    // Rightmost position when expanded to respect granularity

    EAffinity m_affinity;           // the upstream/downstream affinity of the caret
    TextGranularity m_granularity;  // granularity of start/end selection

    // these are cached, can be recalculated by validate()
    SelectionType m_selectionType;    // None, Caret, Range
    bool m_baseIsFirst;               // true if base is before the extent
};

inline bool operator==(const VisibleSelection& a, const VisibleSelection& b)
{
    return a.start() == b.start() && a.end() == b.end() && a.affinity() == b.affinity() && a.granularity() == b.granularity() && a.isBaseFirst() == b.isBaseFirst();
}

inline bool operator!=(const VisibleSelection& a, const VisibleSelection& b)
{
    return !(a == b);
}

} // namespace WebCore

#ifndef NDEBUG
// Outside the WebCore namespace for ease of invocation from gdb.
void showTree(const WebCore::VisibleSelection&);
void showTree(const WebCore::VisibleSelection*);
#endif

#endif // VisibleSelection_h
