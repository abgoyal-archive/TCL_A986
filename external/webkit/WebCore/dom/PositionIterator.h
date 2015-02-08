

#ifndef PositionIterator_h
#define PositionIterator_h

#include "Node.h"
#include "Position.h"

namespace WebCore {

// A Position iterator with constant-time
// increment, decrement, and several predicates on the Position it is at.
// Conversion to/from Position is O(n) in the offset.
class PositionIterator {
public:
    PositionIterator()
        : m_anchorNode(0)
        , m_nodeAfterPositionInAnchor(0)
        , m_offsetInAnchor(0)
    {
    }

    PositionIterator(const Position& pos)
        : m_anchorNode(pos.anchorNode())
        , m_nodeAfterPositionInAnchor(m_anchorNode->childNode(pos.deprecatedEditingOffset()))
        , m_offsetInAnchor(m_nodeAfterPositionInAnchor ? 0 : pos.deprecatedEditingOffset())
    {
    }
    operator Position() const;

    void increment();
    void decrement();

    Node* node() const { return m_anchorNode; }
    int offsetInLeafNode() const { return m_offsetInAnchor; }

    bool atStart() const;
    bool atEnd() const;
    bool atStartOfNode() const;
    bool atEndOfNode() const;
    bool isCandidate() const;

private:
    Node* m_anchorNode;
    Node* m_nodeAfterPositionInAnchor; // If this is non-null, m_nodeAfterPositionInAnchor->parentNode() == m_anchorNode;
    int m_offsetInAnchor;
};

} // namespace WebCore

#endif // PositionIterator_h
