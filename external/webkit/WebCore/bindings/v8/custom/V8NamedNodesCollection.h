

#ifndef V8NamedNodesCollection_h
#define V8NamedNodesCollection_h

#include "Node.h"
#include "NodeList.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class V8NamedNodesCollection : public NodeList {
    public:
        explicit V8NamedNodesCollection(const Vector<RefPtr<Node> >& nodes)
            : m_nodes(nodes) { }
        virtual unsigned length() const { return m_nodes.size(); }
        virtual Node* item(unsigned) const;
        virtual Node* itemWithName(const AtomicString&) const;

    private:
        Vector<RefPtr<Node> > m_nodes;
    };

}  // namespace WebCore

#endif  // V8NamedNodesCollection_h
