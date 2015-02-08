

#ifndef NodeWithIndex_h
#define NodeWithIndex_h

#include "Node.h"

namespace WebCore {

// For use when you want to get the index for a node repeatedly and
// only want to walk the child list to figure out the index once.
class NodeWithIndex {
public:
    NodeWithIndex(Node* node)
        : m_node(node)
        , m_haveIndex(false)
    {
        ASSERT(node);
    }

    Node* node() const { return m_node; }

    int index() const
    {
        if (!m_haveIndex) {
            m_index = m_node->nodeIndex();
            m_haveIndex = true;
        }
        ASSERT(m_index == static_cast<int>(m_node->nodeIndex()));
        return m_index;
    }

private:
    Node* m_node;
    mutable bool m_haveIndex;
    mutable int m_index;
};

}

#endif
