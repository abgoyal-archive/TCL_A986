

#ifndef StaticNodeList_h
#define StaticNodeList_h

#include "NodeList.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class Node;

    class StaticNodeList : public NodeList {
    public:
        // Adopts the contents of the nodes vector.
        static PassRefPtr<StaticNodeList> adopt(Vector<RefPtr<Node> >& nodes)
        {
            return adoptRef(new StaticNodeList(nodes));
        }

        virtual unsigned length() const;
        virtual Node* item(unsigned index) const;
        virtual Node* itemWithName(const AtomicString&) const;

    private:
        StaticNodeList(Vector<RefPtr<Node> >& nodes)
        {
            m_nodes.swap(nodes);
        }
        Vector<RefPtr<Node> > m_nodes;
    };

} // namespace WebCore

#endif // StaticNodeList_h
