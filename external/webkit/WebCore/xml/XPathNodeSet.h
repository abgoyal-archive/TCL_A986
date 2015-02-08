

#ifndef XPathNodeSet_h
#define XPathNodeSet_h

#if ENABLE(XPATH)

#include <wtf/Vector.h>
#include <wtf/Forward.h>

#include "Node.h"

namespace WebCore {

    namespace XPath {

        class NodeSet : public FastAllocBase {
        public:
            NodeSet() : m_isSorted(true), m_subtreesAreDisjoint(false) { }
            
            size_t size() const { return m_nodes.size(); }
            bool isEmpty() const { return !m_nodes.size(); }
            Node* operator[](unsigned i) const { return m_nodes.at(i).get(); }
            void reserveCapacity(size_t newCapacity) { m_nodes.reserveCapacity(newCapacity); }
            void clear() { m_nodes.clear(); }
            void swap(NodeSet& other) { std::swap(m_isSorted, other.m_isSorted); std::swap(m_subtreesAreDisjoint, other.m_subtreesAreDisjoint); m_nodes.swap(other.m_nodes); }

            // NodeSet itself does not verify that nodes in it are unique.
            void append(Node* node) { m_nodes.append(node); }
            void append(PassRefPtr<Node> node) { m_nodes.append(node); }
            void append(const NodeSet& nodeSet) { m_nodes.append(nodeSet.m_nodes); }

            // Returns the set's first node in document order, or 0 if the set is empty.
            Node* firstNode() const;

            // Returns 0 if the set is empty.
            Node* anyNode() const;

            // NodeSet itself doesn't check if it contains nodes in document order - the caller should tell it if it does not.
            void markSorted(bool isSorted) { m_isSorted = isSorted; }
            bool isSorted() const { return m_isSorted || m_nodes.size() < 2; }

            void sort() const;

            // No node in the set is ancestor of another. Unlike m_isSorted, this is assumed to be false, unless the caller sets it to true.
            void markSubtreesDisjoint(bool disjoint) { m_subtreesAreDisjoint = disjoint; }
            bool subtreesAreDisjoint() const { return m_subtreesAreDisjoint || m_nodes.size() < 2; }

            void reverse();
        
        private:
            bool m_isSorted;
            bool m_subtreesAreDisjoint;
            Vector<RefPtr<Node> > m_nodes;
        };

    }
}

#endif // ENABLE(XPATH)
#endif // XPathNodeSet_h
