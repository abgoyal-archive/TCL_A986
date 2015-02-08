

#ifndef Traversal_h
#define Traversal_h

#include "ScriptState.h"
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class Node;
    class NodeFilter;

    class Traversal {
    public:
        Node* root() const { return m_root.get(); }
        unsigned whatToShow() const { return m_whatToShow; }
        NodeFilter* filter() const { return m_filter.get(); }
        bool expandEntityReferences() const { return m_expandEntityReferences; }

    protected:
        Traversal(PassRefPtr<Node>, unsigned whatToShow, PassRefPtr<NodeFilter>, bool expandEntityReferences);
        short acceptNode(ScriptState*, Node*) const;

    private:
        RefPtr<Node> m_root;
        unsigned m_whatToShow;
        RefPtr<NodeFilter> m_filter;
        bool m_expandEntityReferences;
    };

} // namespace WebCore

#endif // Traversal_h
