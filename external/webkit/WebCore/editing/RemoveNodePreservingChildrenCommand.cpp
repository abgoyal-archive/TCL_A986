

#include "config.h"
#include "RemoveNodePreservingChildrenCommand.h"

#include "Node.h"
#include <wtf/Assertions.h>

namespace WebCore {

RemoveNodePreservingChildrenCommand::RemoveNodePreservingChildrenCommand(PassRefPtr<Node> node)
    : CompositeEditCommand(node->document())
    , m_node(node)
{
    ASSERT(m_node);
}

void RemoveNodePreservingChildrenCommand::doApply()
{
    Vector<RefPtr<Node> > children;
    for (Node* child = m_node->firstChild(); child; child = child->nextSibling())
        children.append(child);

    size_t size = children.size();
    for (size_t i = 0; i < size; ++i) {
        RefPtr<Node> child = children[i].release();
        removeNode(child);
        insertNodeBefore(child.release(), m_node);
    }
    removeNode(m_node);
}

}
